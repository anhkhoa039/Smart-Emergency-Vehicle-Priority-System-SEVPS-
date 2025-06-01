import os
import sys
import traci
import sumolib
import numpy as np
from datetime import datetime
from pathlib import Path

# Add project root to Python path
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))

from src.router import EmergencyRouter
from src.controller import TrafficLightController
from src.predictor import CongestionPredictor
from src.v2x import V2XCommunicator
from src.utils.metrics import MetricsCollector
from src.utils.visualization import Visualizer

class SEVPS:
    def __init__(self, sumocfg_path):
        self.sumocfg_path = sumocfg_path
        self.router = EmergencyRouter()
        self.controller = TrafficLightController()
        self.predictor = CongestionPredictor()
        self.v2x = V2XCommunicator()
        self.metrics = MetricsCollector()
        self.visualizer = Visualizer()
        
        # Initialize SUMO
        self.sumo_binary = sumolib.checkBinary('sumo-gui')
        self.sumo_cmd = [self.sumo_binary, '-c', self.sumocfg_path]
        
    def run_simulation(self, max_steps=3600):
        """Run the main simulation loop"""
        traci.start(self.sumo_cmd)
        
        try:
            step = 0
            while step < max_steps:
                traci.simulationStep()
                
                # 1. Detect emergency vehicles
                emergency_vehicles = self._detect_emergency_vehicles()
                
                # 2. Optimize routes for emergency vehicles
                for ev in emergency_vehicles:
                    self.router.optimize_route(ev)
                
                # 3. Update traffic light control
                self.controller.update_traffic_lights(emergency_vehicles)
                
                # 4. Predict congestion
                congestion_map = self.predictor.predict_congestion()
                
                # 5. Handle V2X communication
                self.v2x.process_communications(emergency_vehicles)
                
                # 6. Collect metrics
                self.metrics.update_metrics(step, emergency_vehicles)
                
                # 7. Update visualization
                if step % 10 == 0:  # Update every 10 steps
                    self.visualizer.update_plots(self.metrics.get_current_metrics())
                
                step += 1
                
        finally:
            traci.close()
            self._save_results()
    
    def _detect_emergency_vehicles(self):
        """Detect emergency vehicles in the simulation"""
        emergency_vehicles = []
        for veh_id in traci.vehicle.getIDList():
            if self._is_emergency_vehicle(veh_id):
                emergency_vehicles.append({
                    'id': veh_id,
                    'position': traci.vehicle.getPosition(veh_id),
                    'speed': traci.vehicle.getSpeed(veh_id),
                    'route': traci.vehicle.getRoute(veh_id),
                    'type': traci.vehicle.getTypeID(veh_id)
                })
        return emergency_vehicles
    
    def _is_emergency_vehicle(self, veh_id):
        """Check if a vehicle is an emergency vehicle"""
        veh_type = traci.vehicle.getTypeID(veh_id)
        return veh_type in ['ambulance', 'fire_truck', 'police']
    
    def _save_results(self):
        """Save simulation results and metrics"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        results_dir = project_root / 'results' / timestamp
        results_dir.mkdir(parents=True, exist_ok=True)
        
        # Save metrics
        self.metrics.save_metrics(results_dir)
        # Save visualizations
        self.visualizer.save_plots(results_dir)

if __name__ == "__main__":
    # Path to your SUMO configuration file
    sumocfg_path = str(project_root / "osm.sumocfg")
    
    # Initialize and run the SEVPS
    sevps = SEVPS(sumocfg_path)
    sevps.run_simulation() 