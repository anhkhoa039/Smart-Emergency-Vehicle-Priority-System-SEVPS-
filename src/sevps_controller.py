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
        self.router = EmergencyRouter()
        
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
            "--step-length", "1"  # 100ms step length for responsive control
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
            
        
        if net_file:
            self.network = sumolib.net.readNet('map/'+ net_file)
            print(f"📍 Network loaded: {len(self.network.getNodes())} nodes, {len(self.network.getEdges())} edges")
        else:
            print("⚠️ Warning: Could not find network file in config")
    
    def _setup_traffic_lights(self):
        """Initialize/Collect all traffic lights information in the network"""
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
            
            # Get original route metrics
            current_route = traci.vehicle.getRoute(vehicle_id)
            original_metrics = self.router.get_route_metrics(current_route)
            self.metrics.log_route_metrics(vehicle_id, 'original', original_metrics)
            
            # Optimize route
            emergency_vehicle = {
                'id': vehicle_id,
                'route': current_route,
                'type': traci.vehicle.getTypeID(vehicle_id)
            }
            optimized_route = self.router.optimize_route(emergency_vehicle)
            # optimized_route = None
            
            if optimized_route:
                # Apply the optimized route
                traci.vehicle.setRoute(vehicle_id, optimized_route)
                
                # Get optimized route metrics
                optimized_metrics = self.router.get_route_metrics(optimized_route)
                self.metrics.log_route_metrics(vehicle_id, 'optimized', optimized_metrics)
            
            self.emergency_vehicles[vehicle_id] = EmergencyVehicle(
                vehicle_id=vehicle_id,
                emergency_type=etype,
                priority_level=priority,
                route_id=route_id,
                destination=current_route[-1] if current_route else ""
            )
            
            # Set vehicle color for visual identification
            color_map = {
                EmergencyType.AMBULANCE: (255, 255, 255),  # White
                EmergencyType.FIRE_TRUCK: (255, 0, 0),     # Red
                EmergencyType.POLICE: (0, 0, 255)          # Blue
            }
            traci.vehicle.setColor(vehicle_id, color_map[etype])
            print ("========================================= change color")
            
            print(f"🚨 Registered emergency vehicle: {vehicle_id} ({emergency_type})")
            print(f"   Original route metrics: {original_metrics}")
            if optimized_route:
                print(f"   Optimized route metrics: {optimized_metrics}")
            
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
            traci.vehicle.setLaneChangeMode(vehicle_id, 0b000001) 

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
        
        # Reset all traffic lights to normal operation first
        for tls_id, tls_state in self.traffic_lights.items():
            tls_state.emergency_override = False
        
        # For each emergency vehicle, find the nearest traffic light and preempt only that one
        for ev in active_emergencies.values():
            nearest_tls = self._get_nearest_traffic_light_for_emergency(ev)
            if nearest_tls:
                self._handle_emergency_preemption(nearest_tls, [ev])
        
        # Handle normal operation for all lights not in emergency override
        for tls_id, tls_state in self.traffic_lights.items():
            if not tls_state.emergency_override:
                self._handle_normal_operation(tls_id)
            
            # Update phase timing
            tls_state.time_in_phase = traci.trafficlight.getSpentDuration(tls_id)
    
    def _get_nearest_traffic_light_for_emergency(self, emergency_vehicle: EmergencyVehicle) -> str:
        """Get the nearest traffic light that should be preempted for an emergency vehicle"""
        try:
            candidate_lights = []
            
            for tls_id in self.traffic_lights.keys():
                # Check if this traffic light is on the emergency vehicle's route
                if tls_id not in emergency_vehicle.affected_intersections:
                    continue
                
                tls_position = traci.junction.getPosition(tls_id)
                distance = self._calculate_distance(emergency_vehicle.position, tls_position)
                
                # Check if vehicle is approaching and within detection range
                if (distance <= SEVPSConfig.DETECTION_RANGE and
                    emergency_vehicle.speed > SEVPSConfig.EMERGENCY_SPEED_THRESHOLD):
                    
                    # Calculate ETA (Estimated Time of Arrival)
                    eta = distance / emergency_vehicle.speed if emergency_vehicle.speed > 0 else float('inf')
                    
                    # Only consider for preemption if ETA is within reasonable range
                    # (not too far, not too close - gives time for preemption to take effect)
                    if 2 <= eta <= 30:  # Between 2 and 30 seconds
                        candidate_lights.append({
                            'tls_id': tls_id,
                            'distance': distance,
                            'eta': eta
                        })
            
            if not candidate_lights:
                return None
            
            # Sort by ETA (closest in time gets priority)
            candidate_lights.sort(key=lambda x: x['eta'])
            nearest_tls = candidate_lights[0]['tls_id']
            
            print(f"🚦 Selected nearest TLS {nearest_tls} for {emergency_vehicle.vehicle_id} "
                  f"(ETA: {candidate_lights[0]['eta']:.1f}s, Distance: {candidate_lights[0]['distance']:.1f}m)")
            
            return nearest_tls
            
        except Exception as e:
            print(f"⚠️ Error getting nearest traffic light for {emergency_vehicle.vehicle_id}: {e}")
            return None
    
    def _get_nearby_emergency_vehicles(self, tls_id: str, active_emergencies: Dict[str, EmergencyVehicle]) -> List[EmergencyVehicle]:
        """Get emergency vehicles near a traffic light (updated for improved logic)"""
        nearby = []
        
        try:
            tls_position = traci.junction.getPosition(tls_id)
            
            for ev in active_emergencies.values():
                distance = self._calculate_distance(ev.position, tls_position)
                
                # Check if vehicle is approaching and within detection range
                if (distance <= SEVPSConfig.DETECTION_RANGE and
                    ev.speed > SEVPSConfig.EMERGENCY_SPEED_THRESHOLD and
                    tls_id in ev.affected_intersections):
                    
                    # Calculate ETA
                    eta = distance / ev.speed if ev.speed > 0 else float('inf')
                    
                    # Only include if ETA is within reasonable preemption window
                    if 2 <= eta <= 30:
                        nearby.append(ev)
        
        except Exception as e:
            print(f"⚠️ Error getting nearby vehicles for {tls_id}: {e}")
        
        return sorted(nearby, key=lambda x: x.priority_level, reverse=True)
    
    def _handle_emergency_preemption(self, tls_id: str, emergency_vehicles: List[EmergencyVehicle]):
        """Handle traffic light preemption for emergency vehicles (improved)"""
        tls_state = self.traffic_lights[tls_id]
        highest_priority_ev = emergency_vehicles[0]
        
        try:
            # Calculate required green phase for emergency vehicle
            required_phase = self._calculate_required_phase_fixed(tls_id, highest_priority_ev)
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
            if vehicle_lane == '-518147860#3_0':
                print(f"==== Debugging ==== {vehicle_lane}")
                if vehicle_lane not in controlled_lanes:
                    print("not in controlled lanes")
            return 0  # Default to first phase
            
        except Exception as e:
            print(f"⚠️ Error calculating required phase: {e}")
            return 0
    def _calculate_required_phase_fixed(self, tls_id: str, emergency_vehicle: EmergencyVehicle) -> int:
        """Calculate which phase gives green to emergency vehicle's direction"""
        try:
            # Get vehicle's current lane
            route = traci.vehicle.getRoute(emergency_vehicle.vehicle_id)
            current_index = traci.vehicle.getRouteIndex(emergency_vehicle.vehicle_id)

            # Check next 2 edges in route
            next_edges = route[current_index:current_index + 5]
            controlled_edges = [lane.split('_')[0] for lane in traci.trafficlight.getControlledLanes(tls_id)]

        
            # Find intersection of next edges and controlled edges
            target_edges = set(next_edges) & set(controlled_edges)
            
            if not target_edges:
                return 0
            
            # Use the first matching edge
            target_edge = list(target_edges)[0]
            
            # Get any lane on this edge and find its green phase
            controlled_lanes = traci.trafficlight.getControlledLanes(tls_id)
            for lane in controlled_lanes:
                if lane.startswith(target_edge + '_'):
                    lane_index = controlled_lanes.index(lane)
                    logic = traci.trafficlight.getAllProgramLogics(tls_id)[0]
                    
                    # Return first phase that gives green to any lane on target edge
                    for phase_idx, phase in enumerate(logic.phases):
                        if lane_index < len(phase.state) and phase.state[lane_index] in ['G', 'g']:
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
    
    def _calculate_eta_to_intersection(self, emergency_vehicle: EmergencyVehicle, tls_id: str) -> float:
        """Calculate estimated time of arrival for emergency vehicle to intersection"""
        try:
            tls_position = traci.junction.getPosition(tls_id)
            distance = self._calculate_distance(emergency_vehicle.position, tls_position)
            
            if emergency_vehicle.speed > 0:
                return distance / emergency_vehicle.speed
            else:
                return float('inf')
                
        except Exception as e:
            print(f"⚠️ Error calculating ETA for {emergency_vehicle.vehicle_id} to {tls_id}: {e}")
            return float('inf')
    
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
            # Get current lane of civilian and emergency vehicle
            civilian_lane = traci.vehicle.getLaneIndex(civilian_id)
            ev_lane = traci.vehicle.getLaneIndex(emergency_vehicle.vehicle_id)
            
            # Get positions and calculate relative position
            civilian_pos = traci.vehicle.getPosition(civilian_id)
            ev_pos = traci.vehicle.getPosition(emergency_vehicle.vehicle_id)
            edge_id = traci.vehicle.getRoadID(civilian_id)
            
            # Only suggest lane change if emergency vehicle is behind and approaching
            if not self._is_behind(ev_pos, civilian_pos, edge_id):
                return
            # Check if edge has multiple lanes
            lanes = traci.edge.getLaneNumber(edge_id)
            if lanes < 1:
                return
            # Calculate distance between vehicles
            distance = self._calculate_distance(ev_pos, civilian_pos)
            
            # Only suggest lane change if within V2V range and on same/adjacent lane
            if distance <= SEVPSConfig.V2V_RANGE and abs(civilian_lane - ev_lane) <= 1:
                edge_lanes = traci.edge.getLaneNumber(edge_id)
                
                # Find available lanes that aren't blocked
                available_lanes = []
                for lane_idx in range(edge_lanes):
                    if lane_idx != civilian_lane:  # Don't suggest current lane
                        # Check if lane exists and is not blocked
                        lane_id = f"{edge_id}_{lane_idx}"
                        if traci.lane.getLastStepVehicleNumber(lane_id) < 2:  # Lane has space
                            # Check if lane is not blocked by other vehicles
                            lane_vehicles = traci.lane.getLastStepVehicleIDs(lane_id)
                            if not any(traci.vehicle.getSpeed(vid) < 0.1 for vid in lane_vehicles):
                                available_lanes.append(lane_idx)
                
                if available_lanes:
                    # Choose the lane furthest from emergency vehicle's lane
                    target_lane = max(available_lanes, key=lambda x: abs(x - ev_lane))
                    
                    # Check distance to next intersection
                    next_tls = traci.vehicle.getNextTLS(civilian_id)
                    if not next_tls or next_tls[0][2] > 50:  # At least 50m from next traffic light
                        # Check if vehicle is already in the target lane
                        if civilian_lane != target_lane:
                            # Use lane change mode to encourage lane change
                            traci.vehicle.changeLane(civilian_id, target_lane, 10.0)  # 10 second duration
                            
                            # Visual indicator
                            traci.vehicle.setColor(civilian_id, (0, 255, 255))  # Cyan color temporarily
                            print (f"==== Debugging ==== {civilian_id} changed lane to {target_lane}")
                
        except Exception as e:
            print(f"⚠️ Error sending V2V alert: {e}")
    
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
        self.route_metrics = {
            'original': [],
            'optimized': []
        }
    
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
    
    def log_route_metrics(self, vehicle_id: str, route_type: str, metrics: Dict):
        """Log route performance metrics"""
        metrics['vehicle_id'] = vehicle_id
        metrics['route_type'] = route_type
        self.route_metrics[route_type].append(metrics)
    
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
    
    def generate_route_report(self) -> Dict:
        """Generate route optimization report"""
        if not self.route_metrics['original'] or not self.route_metrics['optimized']:
            return {}
        
        df_original = pd.DataFrame(self.route_metrics['original'])
        df_optimized = pd.DataFrame(self.route_metrics['optimized'])
        
        # Calculate improvements
        improvements = {}
        for metric in ['duration', 'waiting_time', 'time_loss', 'speed']:
            if metric in df_original.columns and metric in df_optimized.columns:
                orig_mean = df_original[metric].mean()
                opt_mean = df_optimized[metric].mean()
                if metric == 'speed':
                    # For speed, higher is better
                    improvement = ((opt_mean - orig_mean) / orig_mean) * 100
                else:
                    # For other metrics, lower is better
                    improvement = ((orig_mean - opt_mean) / orig_mean) * 100
                improvements[f'{metric}_improvement'] = improvement
        
        return improvements

class EmergencyRouter:
    """Advanced route optimization for emergency vehicles"""
    
    def __init__(self):
        """Initialize the emergency router with a directed graph"""
        self.network = nx.DiGraph()
        # Load the network file
        self.net = sumolib.net.readNet('map/osm.net.xml.gz')
        # Network will be built when first needed
        self.nodes = [node.getID() for node in self.net.getNodes()]
    
    def _build_network(self):
        """Build the network graph from SUMO network"""
        # Clear existing network
        self.network.clear()
        
        # Add nodes (junctions)
        for junction in self.net.getNodes():
            self.network.add_node(junction.getID())
        
        # Add edges (roads)
        for edge in self.net.getEdges():
            if edge.allows('passenger'):
                from_node = edge.getFromNode().getID()
                to_node = edge.getToNode().getID()
                length = edge.getLength()
                self.network.add_edge(from_node, to_node, weight=length, edge_id=edge.getID())
        
        print(f"Loaded graph with {len(self.network.nodes())} nodes and {len(self.network.edges())} edges")
    
    def optimize_route(self, emergency_vehicle: Dict) -> Optional[List[str]]:
        """
        Optimize route for an emergency vehicle based on current traffic conditions
        
        Args:
            emergency_vehicle: Dictionary containing vehicle information
            
        Returns:
            List of edge IDs representing the optimized route, or None if no route found
        """
        # Build network if not already built
        if len(self.network.edges) == 0:
            self._build_network()
            
        # Get current position and destination
        current_edge = traci.vehicle.getRoadID(emergency_vehicle['id'])
        if current_edge.startswith(":") or current_edge in self.nodes:
            print("==== [Debug] ==== : Vehicle is in junction")
            return None
        else:
            current_node = self._get_current_node(current_edge, type='from')
        
        route = emergency_vehicle['route']
        destination = route[-1]
        destination_node = self._get_current_node(destination, type='to')
        
        if current_node == destination_node:
            print("==== [Debug] ==== : current_node is the same as destination_node")
            return None
            
        # Update edge weights based on current traffic conditions
        self._update_edge_weights()
        
        try:
            # Find shortest path using Dijkstra's algorithm
            path = nx.shortest_path(
                self.network,
                source=current_node,
                target=destination_node,
                weight='weight'
            )
    
            # Convert node path to edge path
            edge_path = []
            for i in range(len(path) - 1):
                edge = self.network[path[i]][path[i + 1]]['edge_id']
                edge_path.append(edge)
                
            return self.check_and_fix_path([current_edge] + edge_path)
            
        except nx.NetworkXNoPath:
            print(f"No path found for vehicle {emergency_vehicle['id']}")
            return None
    
    def _update_edge_weights(self):
        """Update edge weights based on current traffic conditions"""
        edge_id_list = [edge.getID() for edge in self.net.getEdges()]
        
        for edge_id in edge_id_list:
            # Get current traffic data
            vehicle_count = traci.edge.getLastStepVehicleNumber(edge_id)
            mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
            
            # Get edge from network
            edge = self.net.getEdge(edge_id)
            if edge:
                max_speed = edge.getSpeed()
                
                # Calculate congestion factor
                if mean_speed > 0:
                    congestion_factor = 1 + (vehicle_count * (1 - mean_speed/max_speed))
                else:
                    congestion_factor = float('inf')
                
                # Update edge weight
                base_length = edge.getLength()
                from_node = edge.getFromNode().getID()
                to_node = edge.getToNode().getID()
                
                if self.network.has_edge(from_node, to_node):
                    self.network[from_node][to_node]['weight'] = base_length * congestion_factor
    
    def _get_current_node(self, edge_id, type='from'):
        """Get the node ID for a given edge"""
        edge = self.net.getEdge(edge_id)
        if edge:
            if type == 'from':
                return edge.getToNode().getID()
            elif type == 'to':
                return edge.getToNode().getID()
        return None
    def check_and_fix_path(self, path):
        """Fixes path by inserting any missing edges between disconnected segments.
        
        Args:
            path (List[str]): Original edge path
        Returns:
            fixed_path: list of edges with missing intermediate edges inserted to ensure connectivity.
        """
        if not path:
            return []

        fixed_path = [path[0]]  # Start with the first edge

        for i in range(len(path) - 1):
            from_edge = self.net.getEdge(path[i])
            to_edge = self.net.getEdge(path[i + 1])

            shortest_subpath = [edge.getID() for edge in self.net.getShortestPath(from_edge, to_edge, vClass="passenger")[0]]
            if not shortest_subpath:
                raise ValueError(f"No path found in SUMO between {from_edge} and {to_edge}")

            fixed_path.extend(shortest_subpath[1:])
        
        return fixed_path
    def get_route_metrics(self, route: List[str]) -> Dict[str, float]:
        """
        Calculate metrics for a given route
        
        Args:
            route: List of edge IDs representing the route
            
        Returns:
            Dictionary containing route metrics
        """
        total_length = 0
        total_time = 0
        
        for edge_id in route:
            # Get edge from network
            edge = self.net.getEdge(edge_id)
            if edge:
                # Get edge length
                length = edge.getLength()
                total_length += length
                
                # Estimate time based on current speed
                mean_speed = traci.edge.getLastStepMeanSpeed(edge_id)
                if mean_speed > 0:
                    total_time += length / mean_speed
        
        return {
            'total_length': total_length,
            'estimated_time': total_time,
            'num_edges': len(route)
        }

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
                # Only register if vehicle is actually moving
                if traci.vehicle.getSpeed(vehicle_id) > SEVPSConfig.EMERGENCY_SPEED_THRESHOLD:
                    self.controller.register_emergency_vehicle(vehicle_id, vehicle_type)
                    self.registered_vehicles.add(vehicle_id)
                    
                    # Automatically activate emergency response for all emergency vehicles
                    self.controller.activate_emergency_vehicle(vehicle_id)
                    print(f"🚨 Activated emergency response for: {vehicle_id}")
    
    def run_demo(self, duration_seconds: int = 3600):
        """Run full SEVPS demonstration"""
        print("🚀 Starting SEVPS Demonstration")
        print(f"   Duration: {duration_seconds} seconds")
        print(f"   Emergency vehicles: {len(self.controller.emergency_vehicles)}")
        print(f"   Traffic lights: {len(self.controller.traffic_lights)}")
        
        target_steps = int(duration_seconds / 0.1)  # 0.1 second per step
        last_status = {'emergencies': 0, 'preemptions': 0}  # Track last status
        
        try:
            while self.controller.simulation_step < target_steps:
                # Detect any new emergency vehicles
                self.detect_emergency_vehicles()
                
                # Run simulation step
                self.controller.run_simulation_step()
                
                # Status update every 10 seconds and only if there are changes
                if self.controller.simulation_step % 100 == 0:  # Changed from 50 to 100 (10 seconds)
                    elapsed_time = self.controller.simulation_step * 0.1
                    active_emergencies = sum(1 for ev in self.controller.emergency_vehicles.values() if ev.activated)
                    active_preemptions = sum(1 for tls in self.controller.traffic_lights.values() if tls.emergency_override)
                    
                    # Only print if there are changes in status
                    if (active_emergencies != last_status['emergencies'] or 
                        active_preemptions != last_status['preemptions']):
                        print(f"\n⏱️  Time: {elapsed_time:.1f}s | Active emergencies: {active_emergencies} | Preemptions: {active_preemptions}")
                        
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
                                        if distance <= SEVPSConfig.PREEMPTION_RANGE:
                                            nearby_tls.append(f"{tls_id}({distance:.1f}m)")
                                    
                                    status = "Moving" if ev.speed > SEVPSConfig.EMERGENCY_SPEED_THRESHOLD else "Stopped"
                                    print(f"   - {ev_id}:")
                                    print(f"     Speed: {ev.speed:.1f} m/s ({status})")
                                    print(f"     Position: {ev.current_edge}")
                                    if nearby_tls:
                                        print(f"     Nearby traffic lights: {', '.join(nearby_tls)}")
                                    if ev.affected_intersections:
                                        print(f"     Will affect intersections: {', '.join(ev.affected_intersections)}")
                        
                        # Update last status
                        last_status['emergencies'] = active_emergencies
                        last_status['preemptions'] = active_preemptions
        
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
    SUMO_CONFIG = "map/osm.sumocfg"
    
    try:
        # Initialize SEVPS demo
        demo = SEVPSDemo(SUMO_CONFIG, gui=True)
        
        # Run simulation
        demo.run_demo(duration_seconds=360)  # 1 hour total (matching sumocfg end time)
        
        # Generate and print route optimization report
        route_report = demo.controller.metrics.generate_route_report()
        print("\n📊 Route Optimization Report:")
        for metric, improvement in route_report.items():
            print(f"   {metric}: {improvement:.2f}%")
        
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
    ✅ Automatic emergency vehicle detection from emergency.rou.xml
    ✅ Adaptive traffic light preemption  
    ✅ V2V communication simulation
    ✅ Performance metrics collection
    
    For integration with your existing setup:
    - Use SUMOTrafficController class
    - Emergency vehicles are automatically detected from simulation
    - Call run_simulation_step() in your main loop
    """) 