import traci
import pandas as pd
import numpy as np
from typing import Dict, List
from datetime import datetime
import json

class MetricsCollector:
    def __init__(self):
        self.metrics = {
            'emergency_vehicles': {},
            'traffic_lights': {},
            'general_traffic': {}
        }
        self.history = []
        
    def update_metrics(self, step: int, emergency_vehicles: List[Dict]):
        """Update all metrics for the current simulation step"""
        current_metrics = {
            'step': step,
            'timestamp': datetime.now().isoformat(),
            'emergency_vehicles': self._collect_emergency_metrics(emergency_vehicles),
            'traffic_lights': self._collect_traffic_light_metrics(),
            'general_traffic': self._collect_general_traffic_metrics()
        }
        
        self.history.append(current_metrics)
        self._update_current_metrics(current_metrics)
    
    def _collect_emergency_metrics(self, emergency_vehicles: List[Dict]) -> Dict:
        """Collect metrics for emergency vehicles"""
        metrics = {}
        
        for ev in emergency_vehicles:
            ev_id = ev['id']
            metrics[ev_id] = {
                'position': ev['position'],
                'speed': ev['speed'],
                'type': ev['type'],
                'route': ev['route'],
                'waiting_time': traci.vehicle.getWaitingTime(ev_id),
                'accumulated_waiting_time': traci.vehicle.getAccumulatedWaitingTime(ev_id),
                'time_loss': traci.vehicle.getTimeLoss(ev_id),
                'distance': traci.vehicle.getDistance(ev_id)
            }
        
        return metrics
    
    def _collect_traffic_light_metrics(self) -> Dict:
        """Collect metrics for traffic lights"""
        metrics = {}
        
        for tl_id in traci.trafficlight.getIDList():
            metrics[tl_id] = {
                'current_phase': traci.trafficlight.getPhase(tl_id),
                'phase_duration': traci.trafficlight.getPhaseDuration(tl_id),
                'controlled_lanes': traci.trafficlight.getControlledLanes(tl_id),
                'waiting_vehicles': sum(
                    traci.lane.getLastStepHaltingNumber(lane)
                    for lane in traci.trafficlight.getControlledLanes(tl_id)
                )
            }
        
        return metrics
    
    def _collect_general_traffic_metrics(self) -> Dict:
        """Collect general traffic metrics"""
        return {
            'total_vehicles': traci.vehicle.getIDCount(),
            'average_speed': np.mean([
                traci.vehicle.getSpeed(veh_id)
                for veh_id in traci.vehicle.getIDList()
            ]) if traci.vehicle.getIDCount() > 0 else 0,
            'total_waiting_time': sum(
                traci.vehicle.getWaitingTime(veh_id)
                for veh_id in traci.vehicle.getIDList()
            ),
            'total_time_loss': sum(
                traci.vehicle.getTimeLoss(veh_id)
                for veh_id in traci.vehicle.getIDList()
            )
        }
    
    def _update_current_metrics(self, current_metrics: Dict):
        """Update the current metrics state"""
        self.metrics = current_metrics
    
    def get_current_metrics(self) -> Dict:
        """Get the current metrics state"""
        return self.metrics
    
    def get_emergency_vehicle_metrics(self, vehicle_id: str) -> Dict:
        """Get metrics for a specific emergency vehicle"""
        return self.metrics['emergency_vehicles'].get(vehicle_id, {})
    
    def get_traffic_light_metrics(self, tl_id: str) -> Dict:
        """Get metrics for a specific traffic light"""
        return self.metrics['traffic_lights'].get(tl_id, {})
    
    def get_general_traffic_metrics(self) -> Dict:
        """Get general traffic metrics"""
        return self.metrics['general_traffic']
    
    def save_metrics(self, directory: str):
        """Save all collected metrics to files"""
        # Convert history to DataFrame
        df = pd.DataFrame(self.history)
        
        # Save to CSV
        df.to_csv(f"{directory}/metrics_history.csv", index=False)
        
        # Save current metrics to JSON
        with open(f"{directory}/current_metrics.json", 'w') as f:
            json.dump(self.metrics, f, indent=4)
    
    def calculate_performance_metrics(self) -> Dict:
        """Calculate overall performance metrics"""
        if not self.history:
            return {}
        
        # Calculate average emergency vehicle response time
        ev_response_times = []
        for step_metrics in self.history:
            for ev_metrics in step_metrics['emergency_vehicles'].values():
                ev_response_times.append(ev_metrics['time_loss'])
        
        avg_response_time = np.mean(ev_response_times) if ev_response_times else 0
        
        # Calculate average traffic light efficiency
        tl_efficiencies = []
        for step_metrics in self.history:
            for tl_metrics in step_metrics['traffic_lights'].values():
                waiting_vehicles = tl_metrics['waiting_vehicles']
                total_vehicles = len(tl_metrics['controlled_lanes'])
                if total_vehicles > 0:
                    efficiency = 1 - (waiting_vehicles / total_vehicles)
                    tl_efficiencies.append(efficiency)
        
        avg_tl_efficiency = np.mean(tl_efficiencies) if tl_efficiencies else 0
        
        # Calculate overall traffic flow
        traffic_flows = []
        for step_metrics in self.history:
            traffic_flows.append(step_metrics['general_traffic']['average_speed'])
        
        avg_traffic_flow = np.mean(traffic_flows) if traffic_flows else 0
        
        return {
            'average_emergency_response_time': avg_response_time,
            'average_traffic_light_efficiency': avg_tl_efficiency,
            'average_traffic_flow': avg_traffic_flow,
            'total_simulation_steps': len(self.history)
        } 