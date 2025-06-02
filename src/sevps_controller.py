#!/usr/bin/env python3
"""
Smart Emergency Vehicle Priority System (SEVPS) with SUMO-TraCI Integration
Real-time emergency vehicle prioritization using TraCI control
"""

import os
import sys
import traci
import sumolib
import numpy as np
import pandas as pd
from typing import Dict, List, Tuple, Set, Optional
from dataclasses import dataclass, field
from collections import defaultdict, deque
import time
import xml.etree.ElementTree as ET
from enum import Enum
import threading
import queue
import networkx as nx

# Add SUMO tools to path if needed
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)

# ============================================================================
# CONFIGURATION AND DATA STRUCTURES
# ============================================================================

class EmergencyType(Enum):
    AMBULANCE = "ambulance"
    FIRE_TRUCK = "fire_truck" 
    POLICE = "police"

@dataclass
class EmergencyVehicle:
    """Emergency vehicle data structure"""
    vehicle_id: str
    emergency_type: EmergencyType
    priority_level: float  # 1.0 = highest priority
    route_id: str
    destination: str
    activated: bool = False
    current_edge: str = ""
    position: Tuple[float, float] = (0.0, 0.0)
    speed: float = 0.0
    eta_destination: float = float('inf')
    affected_intersections: Set[str] = field(default_factory=set)
    preemption_history: List[str] = field(default_factory=list)

@dataclass
class TrafficLightState:
    """Traffic light state and control"""
    tls_id: str
    current_phase: int
    phase_duration: int
    time_in_phase: int
    emergency_override: bool = False
    normal_program: str = "0"
    emergency_program: str = "emergency"
    affected_directions: List[int] = field(default_factory=list)
    preemption_queue: deque = field(default_factory=deque)

class SEVPSConfig:
    """Configuration parameters for SEVPS"""
    # Priority weights for different emergency types
    PRIORITY_WEIGHTS = {
        EmergencyType.AMBULANCE: 1.0,
        EmergencyType.FIRE_TRUCK: 0.9,
        EmergencyType.POLICE: 0.8
    }
    
    # Detection and communication ranges
    DETECTION_RANGE = 300.0  # meters
    PREEMPTION_RANGE = 150.0  # meters
    V2V_RANGE = 200.0  # meters
    
    # Timing parameters
    MIN_GREEN_TIME = 5  # minimum green time in seconds
    MAX_GREEN_EXTENSION = 20  # maximum green extension
    YELLOW_TIME = 3  # yellow phase duration
    ALL_RED_TIME = 2  # all-red phase duration
    
    # Performance thresholds
    MAX_CIVILIAN_DELAY = 60  # maximum acceptable civilian delay
    EMERGENCY_SPEED_THRESHOLD = 5.0  # minimum speed for active emergency

# ============================================================================
# SUMO-TRACI INTERFACE AND CONTROL
# ============================================================================

class SUMOTrafficController:
    """Main controller for SUMO traffic simulation with emergency vehicle priority"""
    
    def __init__(self, sumo_config_file: str, gui: bool = False):
        self.config_file = sumo_config_file
        self.gui = gui
        self.emergency_vehicles: Dict[str, EmergencyVehicle] = {}
        self.traffic_lights: Dict[str, TrafficLightState] = {}
        self.civilian_vehicles: Set[str] = set()
        self.network = None
        self.simulation_step = 0
        self.metrics = SEVPSMetrics()
        
        # Initialize SUMO connection
        self._initialize_sumo()
        self._load_network_data()
        self._setup_traffic_lights()
    
    def _initialize_sumo(self):
        """Initialize SUMO simulation"""
        sumo_cmd = []
        if self.gui:
            sumo_cmd = ["sumo-gui", "-c", self.config_file]
        else:
            sumo_cmd = ["sumo", "-c", self.config_file]
        
        sumo_cmd.extend([
            "--waiting-time-memory", "300",
            "--time-to-teleport", "-1",  # Disable teleporting
            "--collision.action", "warn",
            "--step-length", "0.1"  # 100ms step length for responsive control
        ])
        
        traci.start(sumo_cmd)
        print(f"✅ SUMO initialized with config: {self.config_file}")
    
    def _load_network_data(self):
        """Load network topology from SUMO"""
        net_file = None
        
        # Parse config file to find network file
        tree = ET.parse(self.config_file)
        root = tree.getroot()
        for net_tag in root.findall('.//net-file'):
            net_file = net_tag.get('value')
            break
        
        if net_file:
            self.network = sumolib.net.readNet(net_file)
            print(f"📍 Network loaded: {len(self.network.getNodes())} nodes, {len(self.network.getEdges())} edges")
        else:
            print("⚠️ Warning: Could not find network file in config")
    
    def _setup_traffic_lights(self):
        """Initialize all traffic lights in the network"""
        tls_ids = traci.trafficlight.getIDList()
        
        for tls_id in tls_ids:
            current_phase = traci.trafficlight.getPhase(tls_id)
            phase_duration = traci.trafficlight.getPhaseDuration(tls_id)
            
            self.traffic_lights[tls_id] = TrafficLightState(
                tls_id=tls_id,
                current_phase=current_phase,
                phase_duration=int(phase_duration),
                time_in_phase=0
            )
            
            # Create emergency program for this traffic light
            self._create_emergency_program(tls_id)
        
        print(f"🚦 Initialized {len(self.traffic_lights)} traffic lights")
    
    def _create_emergency_program(self, tls_id: str):
        """Create emergency preemption program for traffic light"""
        try:
            # Get current logic
            current_logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
            
            # Create simplified emergency program
            emergency_phases = []
            
            # Emergency green phase (all directions get minimal green)
            for i, phase in enumerate(current_logic.phases):
                if 'G' in phase.state or 'g' in phase.state:
                    # Create emergency version with shorter duration
                    emergency_phase = traci.trafficlight.Phase(
                        duration=10,  # Short duration for quick cycling
                        state=phase.state,
                        minDur=5,
                        maxDur=15
                    )
                    emergency_phases.append(emergency_phase)
            
            # Set emergency program
            emergency_logic = traci.trafficlight.Logic(
                programID="emergency",
                type=0,
                currentPhaseIndex=0,
                phases=emergency_phases
            )
            
            traci.trafficlight.setProgramLogic(tls_id, emergency_logic)
            traci.trafficlight.setProgram(tls_id, "0")  # Start with normal program
            
        except Exception as e:
            print(f"⚠️ Could not create emergency program for {tls_id}: {e}")
    
    def register_emergency_vehicle(self, vehicle_id: str, emergency_type: str, route_id: str = None):
        """Register a vehicle as emergency vehicle"""
        try:
            etype = EmergencyType(emergency_type.lower())
            priority = SEVPSConfig.PRIORITY_WEIGHTS[etype]
            
            # Get route info if available
            if route_id is None:
                route_id = traci.vehicle.getRouteID(vehicle_id)
            
            self.emergency_vehicles[vehicle_id] = EmergencyVehicle(
                vehicle_id=vehicle_id,
                emergency_type=etype,
                priority_level=priority,
                route_id=route_id,
                destination=traci.vehicle.getRoute(vehicle_id)[-1] if traci.vehicle.getRoute(vehicle_id) else ""
            )
            
            # Set vehicle color for visual identification
            color_map = {
                EmergencyType.AMBULANCE: (255, 255, 255),  # White
                EmergencyType.FIRE_TRUCK: (255, 0, 0),     # Red
                EmergencyType.POLICE: (0, 0, 255)          # Blue
            }
            traci.vehicle.setColor(vehicle_id, color_map[etype])
            
            print(f"🚨 Registered emergency vehicle: {vehicle_id} ({emergency_type})")
            
        except Exception as e:
            print(f"❌ Error registering emergency vehicle {vehicle_id}: {e}")
    
    def activate_emergency_vehicle(self, vehicle_id: str):
        """Activate emergency response for a vehicle"""
        if vehicle_id in self.emergency_vehicles:
            self.emergency_vehicles[vehicle_id].activated = True
            self._calculate_affected_intersections(vehicle_id)
            print(f"🚨 EMERGENCY ACTIVATED: {vehicle_id}")
            
            # Change vehicle appearance for active emergency
            traci.vehicle.setSpeedMode(vehicle_id, 0b011110)  # More aggressive driving
            self.metrics.log_emergency_activation(vehicle_id, self.simulation_step)
    
    def _calculate_affected_intersections(self, vehicle_id: str):
        """Calculate which intersections will be affected by emergency vehicle"""
        if vehicle_id not in self.emergency_vehicles:
            return
        
        ev = self.emergency_vehicles[vehicle_id]
        try:
            # Get remaining route
            remaining_route = traci.vehicle.getRoute(vehicle_id)
            route_index = traci.vehicle.getRouteIndex(vehicle_id)
            remaining_edges = remaining_route[route_index:]
            
            # Find traffic lights on remaining route
            affected_tls = set()
            for edge_id in remaining_edges:
                if self.network:
                    edge = self.network.getEdge(edge_id)
                    to_node = edge.getToNode()
                    
                    # Check if node has traffic light
                    for tls_id in self.traffic_lights.keys():
                        tls_edges = set(traci.trafficlight.getControlledLanes(tls_id))
                        edge_lanes = set([edge_id + "_" + str(i) for i in range(edge.getLaneNumber())])
                        
                        if tls_edges.intersection(edge_lanes):
                            affected_tls.add(tls_id)
            
            ev.affected_intersections = affected_tls
            print(f"📍 Vehicle {vehicle_id} will affect {len(affected_tls)} intersections")
            
        except Exception as e:
            print(f"⚠️ Error calculating affected intersections for {vehicle_id}: {e}")
    
    def update_vehicle_states(self):
        """Update all vehicle states and positions"""
        # Update emergency vehicles
        for vehicle_id, ev in self.emergency_vehicles.items():
            if traci.vehicle.getIDList().__contains__(vehicle_id):
                ev.position = traci.vehicle.getPosition(vehicle_id)
                ev.speed = traci.vehicle.getSpeed(vehicle_id)
                ev.current_edge = traci.vehicle.getRoadID(vehicle_id)
                
                # Calculate ETA to destination
                if ev.activated and ev.speed > 0:
                    remaining_distance = traci.vehicle.getDistance(vehicle_id)
                    ev.eta_destination = remaining_distance / ev.speed
        
        # Track civilian vehicles
        all_vehicles = set(traci.vehicle.getIDList())
        self.civilian_vehicles = all_vehicles - set(self.emergency_vehicles.keys())
    
    def process_traffic_light_control(self):
        """Main traffic light control logic with emergency prioritization"""
        active_emergencies = {vid: ev for vid, ev in self.emergency_vehicles.items() 
                            if ev.activated and traci.vehicle.getIDList().__contains__(vid)}
        
        for tls_id, tls_state in self.traffic_lights.items():
            # Check for nearby emergency vehicles
            nearby_emergencies = self._get_nearby_emergency_vehicles(tls_id, active_emergencies)
            
            if nearby_emergencies:
                self._handle_emergency_preemption(tls_id, nearby_emergencies)
            else:
                self._handle_normal_operation(tls_id)
            
            # Update phase timing
            tls_state.time_in_phase = traci.trafficlight.getSpentDuration(tls_id)
    
    def _get_nearby_emergency_vehicles(self, tls_id: str, active_emergencies: Dict[str, EmergencyVehicle]) -> List[EmergencyVehicle]:
        """Get emergency vehicles near a traffic light"""
        nearby = []
        
        try:
            tls_position = traci.junction.getPosition(tls_id)
            
            for ev in active_emergencies.values():
                # Skip if vehicle is not moving
                if ev.speed < SEVPSConfig.EMERGENCY_SPEED_THRESHOLD:
                    continue
                    
                distance = self._calculate_distance(ev.position, tls_position)
                
                # Check if vehicle is approaching and within detection range
                if distance <= SEVPSConfig.DETECTION_RANGE:
                    # Check if vehicle is actually approaching the intersection
                    if self._is_approaching_intersection(ev, tls_id):
                        nearby.append(ev)
                        print(f"🚦 Emergency vehicle {ev.vehicle_id} approaching {tls_id} at distance {distance:.1f}m")
        
        except Exception as e:
            print(f"⚠️ Error getting nearby vehicles for {tls_id}: {e}")
        
        return sorted(nearby, key=lambda x: x.priority_level, reverse=True)
    
    def _is_approaching_intersection(self, ev: EmergencyVehicle, tls_id: str) -> bool:
        """Check if emergency vehicle is actually approaching the intersection"""
        try:
            # Get the current edge and next edge in route
            current_edge = ev.current_edge
            
            # Skip if vehicle is in a junction (edge ID starts with ':')
            if current_edge.startswith(':'):
                return False
                
            route_index = traci.vehicle.getRouteIndex(ev.vehicle_id)
            route = traci.vehicle.getRoute(ev.vehicle_id)
            
            if route_index < len(route) - 1:
                next_edge = route[route_index + 1]
                
                # Skip if next edge is a junction
                if next_edge.startswith(':'):
                    return False
                
                # Get the junction between current and next edge
                current_edge_obj = self.network.getEdge(current_edge)
                next_edge_obj = self.network.getEdge(next_edge)
                
                if current_edge_obj and next_edge_obj:
                    junction = current_edge_obj.getToNode()
                    if junction.getID() == tls_id:
                        return True
            
            return False
            
        except Exception as e:
            # Only print error if it's not a junction-related error
            if not str(e).startswith(':'):
                print(f"⚠️ Error checking intersection approach: {e}")
            return False
    
    def _handle_emergency_preemption(self, tls_id: str, emergency_vehicles: List[EmergencyVehicle]):
        """Handle traffic light preemption for emergency vehicles"""
        tls_state = self.traffic_lights[tls_id]
        highest_priority_ev = emergency_vehicles[0]
        
        try:
            # Calculate required green phase for emergency vehicle
            required_phase = self._calculate_required_phase(tls_id, highest_priority_ev)
            current_phase = traci.trafficlight.getPhase(tls_id)
            
            # Check if preemption is needed
            if required_phase != current_phase or not tls_state.emergency_override:
                # Activate emergency override
                if not tls_state.emergency_override:
                    tls_state.emergency_override = True
                    self.metrics.log_preemption_event(tls_id, highest_priority_ev.vehicle_id, self.simulation_step)
                    print(f"🚦 PREEMPTION: {tls_id} for {highest_priority_ev.vehicle_id}")
                
                # Set appropriate phase
                self._set_emergency_phase(tls_id, required_phase)
                
                # Add to vehicle's preemption history
                if tls_id not in highest_priority_ev.preemption_history:
                    highest_priority_ev.preemption_history.append(tls_id)
        
        except Exception as e:
            print(f"⚠️ Error in emergency preemption for {tls_id}: {e}")
    
    def _calculate_required_phase(self, tls_id: str, emergency_vehicle: EmergencyVehicle) -> int:
        """Calculate which phase gives green to emergency vehicle's direction"""
        try:
            # Get vehicle's current lane
            vehicle_lane = traci.vehicle.getLaneID(emergency_vehicle.vehicle_id)
            
            # Get all phases and find which one gives green to this lane
            logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
            
            if vehicle_lane in controlled_lanes:
                lane_index = controlled_lanes.index(vehicle_lane)
                
                # Find phase that gives green (G or g) to this lane
                for phase_idx, phase in enumerate(logic.phases):
                    if phase_idx < len(phase.state) and phase.state[lane_index] in ['G', 'g']:
                        return phase_idx
            
            return 0  # Default to first phase
            
        except Exception as e:
            print(f"⚠️ Error calculating required phase: {e}")
            return 0
    
    def _set_emergency_phase(self, tls_id: str, phase: int):
        """Set traffic light to emergency phase"""
        try:
            # Set the specific phase
            traci.trafficlight.setPhase(tls_id, phase)
            
            # Extend green time for emergency vehicle
            traci.trafficlight.setPhaseDuration(tls_id, SEVPSConfig.MAX_GREEN_EXTENSION)
            
        except Exception as e:
            print(f"⚠️ Error setting emergency phase: {e}")
    
    def _handle_normal_operation(self, tls_id: str):
        """Handle normal traffic light operation when no emergencies nearby"""
        tls_state = self.traffic_lights[tls_id]
        
        if tls_state.emergency_override:
            # Return to normal operation
            tls_state.emergency_override = False
            traci.trafficlight.setProgram(tls_id, tls_state.normal_program)
            print(f"🚦 NORMAL: {tls_id} returned to normal operation")
    
    def send_v2v_alerts(self):
        """Send V2V alerts to civilian vehicles about approaching emergency vehicles"""
        active_emergencies = {vid: ev for vid, ev in self.emergency_vehicles.items() 
                            if ev.activated and traci.vehicle.getIDList().__contains__(vid)}
        
        for ev in active_emergencies.values():
            nearby_civilians = self._get_nearby_civilian_vehicles(ev)
            
            for civilian_id in nearby_civilians:
                self._send_lane_change_suggestion(civilian_id, ev)
    
    def _get_nearby_civilian_vehicles(self, emergency_vehicle: EmergencyVehicle) -> List[str]:
        """Get civilian vehicles near emergency vehicle"""
        nearby = []
        
        for civilian_id in self.civilian_vehicles:
            if traci.vehicle.getIDList().__contains__(civilian_id):
                civilian_pos = traci.vehicle.getPosition(civilian_id)
                distance = self._calculate_distance(emergency_vehicle.position, civilian_pos)
                
                if distance <= SEVPSConfig.V2V_RANGE:
                    nearby.append(civilian_id)
        
        return nearby
    
    def _send_lane_change_suggestion(self, civilian_id: str, emergency_vehicle: EmergencyVehicle):
        """Suggest lane change to civilian vehicle to make way for emergency vehicle"""
        try:
            # Get current lane and road information
            current_lane = traci.vehicle.getLaneIndex(civilian_id)
            edge_id = traci.vehicle.getRoadID(civilian_id)
            num_lanes = traci.edge.getLaneNumber(edge_id)
            
            # Get emergency vehicle's lane
            ev_lane = traci.vehicle.getLaneIndex(emergency_vehicle.vehicle_id)
            ev_edge = traci.vehicle.getRoadID(emergency_vehicle.vehicle_id)
            
            # Only suggest lane change if on the same road and emergency vehicle is approaching
            if edge_id == ev_edge:
                # Calculate relative position
                ev_pos = traci.vehicle.getPosition(emergency_vehicle.vehicle_id)
                civ_pos = traci.vehicle.getPosition(civilian_id)
                
                # Only suggest lane change if emergency vehicle is behind
                if self._is_behind(ev_pos, civ_pos, edge_id):
                    # Find available lanes
                    available_lanes = []
                    for lane_idx in range(num_lanes):
                        if lane_idx != current_lane:  # Don't suggest current lane
                            # Check if lane exists and is not blocked
                            lane_id = f"{edge_id}_{lane_idx}"
                            if traci.lane.getLastStepVehicleNumber(lane_id) < 2:  # Lane has space
                                available_lanes.append(lane_idx)
                    
                    if available_lanes:
                        # Choose the lane furthest from emergency vehicle's lane
                        target_lane = max(available_lanes, key=lambda x: abs(x - ev_lane))
                        
                        # Only change lane if not already changing
                        if not traci.vehicle.isChangingLane(civilian_id):
                            # Use lane change mode to encourage lane change
                            traci.vehicle.changeLane(civilian_id, target_lane, 5.0)  # 5 second duration
                            
                            # Visual indicator
                            traci.vehicle.setColor(civilian_id, (255, 255, 0))  # Yellow color temporarily
                
        except Exception as e:
            print(f"⚠️ Error sending V2V alert: {e}")
    
    def _is_behind(self, pos1: Tuple[float, float], pos2: Tuple[float, float], edge_id: str) -> bool:
        """Check if pos1 is behind pos2 on the same edge"""
        try:
            # Get edge shape
            edge_shape = traci.edge.getShape(edge_id)
            if len(edge_shape) < 2:
                return False
                
            # Calculate distances along edge
            dist1 = traci.simulation.getDistance2D(pos1[0], pos1[1], edge_shape[0][0], edge_shape[0][1])
            dist2 = traci.simulation.getDistance2D(pos2[0], pos2[1], edge_shape[0][0], edge_shape[0][1])
            
            return dist1 < dist2
            
        except Exception:
            return False
    
    def _calculate_distance(self, pos1: Tuple[float, float], pos2: Tuple[float, float]) -> float:
        """Calculate Euclidean distance between two positions"""
        return np.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def run_simulation_step(self):
        """Execute one simulation step"""
        traci.simulationStep()
        self.simulation_step += 1
        
        # Update all vehicle states
        self.update_vehicle_states()
        
        # Process traffic light control
        self.process_traffic_light_control()
        
        # Send V2V alerts
        self.send_v2v_alerts()
        
        # Collect metrics
        self.metrics.collect_step_metrics(self.simulation_step, self.emergency_vehicles, self.traffic_lights)
    
    def close(self):
        """Close SUMO simulation"""
        traci.close()
        print("🔚 SUMO simulation closed")

# ============================================================================
# METRICS AND PERFORMANCE TRACKING
# ============================================================================

class SEVPSMetrics:
    """Performance metrics collection for SEVPS"""
    
    def __init__(self):
        self.emergency_activations = []
        self.preemption_events = []
        self.response_times = []
        self.civilian_delays = []
        self.step_data = []
    
    def log_emergency_activation(self, vehicle_id: str, step: int):
        """Log emergency vehicle activation"""
        self.emergency_activations.append({
            'vehicle_id': vehicle_id,
            'activation_step': step,
            'activation_time': step * 0.1  # Convert to seconds
        })
    
    def log_preemption_event(self, tls_id: str, vehicle_id: str, step: int):
        """Log traffic light preemption event"""
        self.preemption_events.append({
            'tls_id': tls_id,
            'vehicle_id': vehicle_id,
            'step': step,
            'time': step * 0.1
        })
    
    def collect_step_metrics(self, step: int, emergency_vehicles: Dict, traffic_lights: Dict):
        """Collect metrics for current simulation step"""
        active_emergencies = sum(1 for ev in emergency_vehicles.values() if ev.activated)
        active_preemptions = sum(1 for tls in traffic_lights.values() if tls.emergency_override)
        
        self.step_data.append({
            'step': step,
            'time': step * 0.1,
            'active_emergencies': active_emergencies,
            'active_preemptions': active_preemptions
        })
    
    def generate_report(self) -> Dict:
        """Generate comprehensive performance report"""
        df_steps = pd.DataFrame(self.step_data)
        df_preemptions = pd.DataFrame(self.preemption_events)
        
        report = {
            'simulation_duration': len(self.step_data) * 0.1,
            'total_emergency_activations': len(self.emergency_activations),
            'total_preemption_events': len(self.preemption_events),
            'average_active_emergencies': df_steps['active_emergencies'].mean() if not df_steps.empty else 0,
            'preemption_rate': len(self.preemption_events) / max(1, len(self.emergency_activations)),
            'unique_intersections_preempted': len(df_preemptions['tls_id'].unique()) if not df_preemptions.empty else 0
        }
        
        return report

# ============================================================================
# MAIN EXECUTION AND TESTING
# ============================================================================

class SEVPSDemo:
    """Demonstration and testing class for SEVPS"""
    
    def __init__(self, config_file: str, gui: bool = True):
        self.controller = SUMOTrafficController(config_file, gui)
        self.registered_vehicles = set()  # Track registered vehicles
    
    def detect_emergency_vehicles(self):
        """Detect and register emergency vehicles from the simulation"""
        vehicle_list = traci.vehicle.getIDList()
        for vehicle_id in vehicle_list:
            # Skip if already registered
            if vehicle_id in self.registered_vehicles:
                continue
                
            # Check if vehicle is an emergency vehicle based on type
            vehicle_type = traci.vehicle.getTypeID(vehicle_id)
            if vehicle_type in ['ambulance', 'fire_truck', 'police']:
                self.controller.register_emergency_vehicle(vehicle_id, vehicle_type)
                self.registered_vehicles.add(vehicle_id)
                print(f"🚨 Registered emergency vehicle: {vehicle_id} ({vehicle_type})")
                
                # Automatically activate the emergency vehicle
                self.controller.activate_emergency_vehicle(vehicle_id)
                print(f"🚨 Activated emergency response for: {vehicle_id}")
    
    def run_demo(self, duration_seconds: int = 3600):
        """Run full SEVPS demonstration"""
        print("🚀 Starting SEVPS Demonstration")
        print(f"   Duration: {duration_seconds} seconds")
        
        target_steps = int(duration_seconds / 0.1)  # 0.1 second per step
        
        try:
            while self.controller.simulation_step < target_steps:
                # Detect any new emergency vehicles
                self.detect_emergency_vehicles()
                
                # Run simulation step
                self.controller.run_simulation_step()
                
                # Status update every 5 seconds
                if self.controller.simulation_step % 50 == 0:
                    elapsed_time = self.controller.simulation_step * 0.1
                    active_emergencies = sum(1 for ev in self.controller.emergency_vehicles.values() if ev.activated)
                    active_preemptions = sum(1 for tls in self.controller.traffic_lights.values() if tls.emergency_override)
                    
                    print(f"⏱️  Time: {elapsed_time:.1f}s | Active emergencies: {active_emergencies} | Preemptions: {active_preemptions}")
                    
                    # Print more detailed status for active emergencies
                    if active_emergencies > 0:
                        print("\nActive Emergency Vehicles:")
                        for ev_id, ev in self.controller.emergency_vehicles.items():
                            if ev.activated:
                                # Get nearby traffic lights
                                nearby_tls = []
                                for tls_id in self.controller.traffic_lights:
                                    tls_pos = traci.junction.getPosition(tls_id)
                                    distance = self.controller._calculate_distance(ev.position, tls_pos)
                                    if distance <= SEVPSConfig.PREEMPTION_RANGE:  # Use PREEMPTION_RANGE instead of DETECTION_RANGE
                                        nearby_tls.append(f"{tls_id}({distance:.1f}m)")
                                
                                status = "Moving" if ev.speed > SEVPSConfig.EMERGENCY_SPEED_THRESHOLD else "Stopped"
                                print(f"   - {ev_id}:")
                                print(f"     Speed: {ev.speed:.1f} m/s ({status})")
                                print(f"     Position: {ev.current_edge}")
                                if nearby_tls:
                                    print(f"     Nearby traffic lights: {', '.join(nearby_tls)}")
                                if ev.affected_intersections:
                                    print(f"     Will affect intersections: {', '.join(ev.affected_intersections)}")
        
        except KeyboardInterrupt:
            print("\n🛑 Simulation interrupted by user")
        
        # Generate final report
        report = self.controller.metrics.generate_report()
        print("\n📊 SEVPS Performance Report:")
        for key, value in report.items():
            print(f"   {key}: {value}")
        
        return report
    
    def close(self):
        """Close demonstration"""
        self.controller.close()

def run_sevps_example():
    """Example usage of SEVPS with SUMO"""
    
    # Configuration - Using the user's SUMO config file
    SUMO_CONFIG = "osm.sumocfg"
    
    try:
        # Initialize SEVPS demo
        demo = SEVPSDemo(SUMO_CONFIG, gui=True)
        
        # Run simulation
        demo.run_demo(duration_seconds=3600)  # 1 hour total (matching sumocfg end time)
        
    except Exception as e:
        print(f"❌ Error running SEVPS demo: {e}")
    
    finally:
        demo.close()

if __name__ == "__main__":
    # Example usage
    run_sevps_example()
    
    print("""
    🚨 SEVPS SUMO-TraCI Integration Ready!
    
    To use this system:
    1. Make sure osm.sumocfg and related files are in the current directory
    2. Run: python sevps_controller.py
    
    Key Features:
    ✅ Automatic emergency vehicle detection
    ✅ Adaptive traffic light preemption  
    ✅ V2V communication simulation
    ✅ Performance metrics collection
    
    For integration with your existing setup:
    - Use SUMOTrafficController class
    - Emergency vehicles are automatically detected from simulation
    - Call run_simulation_step() in your main loop
    """) 