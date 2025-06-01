import traci
import numpy as np
from typing import Dict, List, Set
import math

class V2XCommunicator:
    def __init__(self, communication_range=200):
        self.communication_range = communication_range
        self.vehicle_registry = {}  # Store vehicle information
        self.infrastructure_registry = {}  # Store infrastructure information
        
    def process_communications(self, emergency_vehicles: List[Dict]):
        """Process V2X communications for all vehicles"""
        # Update vehicle registry
        self._update_vehicle_registry()
        
        # Update infrastructure registry
        self._update_infrastructure_registry()
        
        # Process V2V communications
        self._process_v2v_communications(emergency_vehicles)
        
        # Process V2I communications
        self._process_v2i_communications(emergency_vehicles)
    
    def _update_vehicle_registry(self):
        """Update information about all vehicles in the simulation"""
        for veh_id in traci.vehicle.getIDList():
            self.vehicle_registry[veh_id] = {
                'position': traci.vehicle.getPosition(veh_id),
                'speed': traci.vehicle.getSpeed(veh_id),
                'type': traci.vehicle.getTypeID(veh_id),
                'route': traci.vehicle.getRoute(veh_id),
                'lane': traci.vehicle.getLaneID(veh_id)
            }
    
    def _update_infrastructure_registry(self):
        """Update information about traffic infrastructure"""
        # Update traffic lights
        for tl_id in traci.trafficlight.getIDList():
            self.infrastructure_registry[tl_id] = {
                'type': 'traffic_light',
                'position': traci.junction.getPosition(tl_id),
                'controlled_lanes': traci.trafficlight.getControlledLanes(tl_id),
                'current_phase': traci.trafficlight.getPhase(tl_id)
            }
    
    def _process_v2v_communications(self, emergency_vehicles: List[Dict]):
        """Process vehicle-to-vehicle communications"""
        for ev in emergency_vehicles:
            ev_position = ev['position']
            ev_id = ev['id']
            
            # Find nearby vehicles
            nearby_vehicles = self._find_nearby_vehicles(ev_position)
            
            # Send warning to nearby vehicles
            for nearby_veh_id in nearby_vehicles:
                if self._should_warn_vehicle(nearby_veh_id, ev):
                    self._send_vehicle_warning(nearby_veh_id, ev)
    
    def _process_v2i_communications(self, emergency_vehicles: List[Dict]):
        """Process vehicle-to-infrastructure communications"""
        for ev in emergency_vehicles:
            ev_position = ev['position']
            
            # Find nearby infrastructure
            nearby_infrastructure = self._find_nearby_infrastructure(ev_position)
            
            # Send priority request to traffic lights
            for infra_id in nearby_infrastructure:
                if self.infrastructure_registry[infra_id]['type'] == 'traffic_light':
                    self._send_priority_request(infra_id, ev)
    
    def _find_nearby_vehicles(self, position: tuple) -> Set[str]:
        """Find vehicles within communication range"""
        nearby_vehicles = set()
        
        for veh_id, veh_data in self.vehicle_registry.items():
            distance = self._calculate_distance(position, veh_data['position'])
            if distance <= self.communication_range:
                nearby_vehicles.add(veh_id)
        
        return nearby_vehicles
    
    def _find_nearby_infrastructure(self, position: tuple) -> Set[str]:
        """Find infrastructure within communication range"""
        nearby_infrastructure = set()
        
        for infra_id, infra_data in self.infrastructure_registry.items():
            distance = self._calculate_distance(position, infra_data['position'])
            if distance <= self.communication_range:
                nearby_infrastructure.add(infra_id)
        
        return nearby_infrastructure
    
    def _should_warn_vehicle(self, vehicle_id: str, emergency_vehicle: Dict) -> bool:
        """Determine if a vehicle should be warned about the emergency vehicle"""
        veh_data = self.vehicle_registry[vehicle_id]
        
        # Don't warn other emergency vehicles
        if veh_data['type'] in ['ambulance', 'fire_truck', 'police']:
            return False
        
        # Check if vehicles are on the same road or nearby roads
        ev_lane = emergency_vehicle['lane']
        veh_lane = veh_data['lane']
        
        return self._are_lanes_connected(ev_lane, veh_lane)
    
    def _send_vehicle_warning(self, vehicle_id: str, emergency_vehicle: Dict):
        """Send warning to a vehicle about an approaching emergency vehicle"""
        # Get vehicle's current speed and position
        current_speed = traci.vehicle.getSpeed(vehicle_id)
        current_position = traci.vehicle.getPosition(vehicle_id)
        
        # Calculate distance to emergency vehicle
        distance = self._calculate_distance(current_position, emergency_vehicle['position'])
        
        # Adjust speed based on distance
        if distance < 50:  # Very close
            target_speed = current_speed * 0.5  # Slow down significantly
        elif distance < 100:  # Moderately close
            target_speed = current_speed * 0.7  # Slow down moderately
        else:  # Far away
            target_speed = current_speed * 0.9  # Slight slowdown
        
        # Apply speed change
        traci.vehicle.setSpeed(vehicle_id, target_speed)
    
    def _send_priority_request(self, tl_id: str, emergency_vehicle: Dict):
        """Send priority request to a traffic light"""
        # Get current phase and timing
        current_phase = traci.trafficlight.getPhase(tl_id)
        current_duration = traci.trafficlight.getPhaseDuration(tl_id)
        
        # Calculate time to reach intersection
        distance = self._calculate_distance(
            emergency_vehicle['position'],
            self.infrastructure_registry[tl_id]['position']
        )
        speed = emergency_vehicle['speed']
        time_to_reach = distance / speed if speed > 0 else float('inf')
        
        # If emergency vehicle is close, prioritize its direction
        if time_to_reach < 10:  # Less than 10 seconds to reach
            # Find the phase that gives priority to the emergency vehicle's direction
            priority_phase = self._find_priority_phase(tl_id, emergency_vehicle)
            
            if priority_phase != current_phase:
                # Switch to priority phase
                traci.trafficlight.setPhase(tl_id, priority_phase)
                # Set shorter duration for priority phase
                traci.trafficlight.setPhaseDuration(tl_id, 5)
    
    def _find_priority_phase(self, tl_id: str, emergency_vehicle: Dict) -> int:
        """Find the traffic light phase that gives priority to the emergency vehicle"""
        # Get the lane the emergency vehicle is on
        ev_lane = emergency_vehicle['lane']
        
        # Get all phases for this traffic light
        phases = traci.trafficlight.getAllProgramLogics(tl_id)[0].phases
        
        # Find the phase that gives green to the emergency vehicle's direction
        for i, phase in enumerate(phases):
            if self._is_lane_green_in_phase(tl_id, ev_lane, phase):
                return i
        
        return 0  # Default to first phase if no match found
    
    def _is_lane_green_in_phase(self, tl_id: str, lane_id: str, phase) -> bool:
        """Check if a lane has green light in a given phase"""
        # Get the index of the lane in the traffic light's controlled lanes
        controlled_lanes = self.infrastructure_registry[tl_id]['controlled_lanes']
        if lane_id not in controlled_lanes:
            return False
        
        lane_index = controlled_lanes.index(lane_id)
        
        # Check if the lane has green light in this phase
        return phase.state[lane_index] == 'G'
    
    def _calculate_distance(self, pos1: tuple, pos2: tuple) -> float:
        """Calculate Euclidean distance between two positions"""
        return math.sqrt(
            (pos1[0] - pos2[0])**2 +
            (pos1[1] - pos2[1])**2
        )
    
    def _are_lanes_connected(self, lane1: str, lane2: str) -> bool:
        """Check if two lanes are connected or nearby"""
        # Get the edges the lanes belong to
        edge1 = lane1.split('_')[0]
        edge2 = lane2.split('_')[0]
        
        # Check if edges are connected
        return (edge1 == edge2 or  # Same road
                traci.edge.getToNode(edge1) == traci.edge.getFromNode(edge2) or  # Connected
                traci.edge.getFromNode(edge1) == traci.edge.getToNode(edge2))  # Connected in reverse 