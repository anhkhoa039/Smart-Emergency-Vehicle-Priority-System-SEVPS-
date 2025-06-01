import os
import sys
import traci
import sumolib
import numpy as np
from datetime import datetime
from pathlib import Path
import traceback

# Add project root to Python path
project_root = Path(__file__).parent.parent
sys.path.append(str(project_root))

from src.router import EmergencyRouter
from src.controller import TrafficLightController
from src.predictor import CongestionPredictor
from src.v2x import V2XCommunicator
from src.utils.metrics import MetricsCollector
from src.utils.visualization import Visualizer

class SEVPSTester:
    def __init__(self, sumocfg_path):
        print("\n=== Initializing SEVPSTester ===")
        self.sumocfg_path = sumocfg_path
        
        try:
            # Initialize SUMO first
            print("✓ Checking SUMO binary...")
            self.sumo_binary = sumolib.checkBinary('sumo-gui')
            self.sumo_cmd = [self.sumo_binary, '-c', self.sumocfg_path]
            
            # Start TraCI connection
            print("✓ Starting TraCI connection...")
            traci.start(self.sumo_cmd)
            
            # Initialize components after TraCI connection is established
            print("✓ Initializing components...")
            self.router = EmergencyRouter()
            self.controller = TrafficLightController()
            self.predictor = CongestionPredictor()
            self.v2x = V2XCommunicator()
            self.metrics = MetricsCollector()
            self.visualizer = Visualizer()
            
            # Test results
            self.test_results = {
                'emergency_detection': False,
                'route_optimization': False,
                'traffic_light_control': False,
                'congestion_prediction': False,
                'v2x_communication': False,
                'metrics_collection': False,
                'visualization': False
            }
            print("✓ Initialization complete!\n")
            
        except Exception as e:
            print(f"\n❌ Error during initialization:")
            print(f"Error type: {type(e).__name__}")
            print(f"Error message: {str(e)}")
            raise
    
    def run_tests(self, max_steps=3600):
        """Run all system tests"""
        print("\n=== Starting SEVPS System Tests ===")
        print(f"Simulation will run for {max_steps} steps")
        
        try:
            step = 0
            while step < max_steps and traci.simulation.getMinExpectedNumber() > 0:
                try:
                    # Perform simulation step
                    traci.simulationStep()
                    
                    # Test emergency vehicle detection
                    emergency_vehicles = self._test_emergency_detection()
                    
                    # Test route optimization
                    if emergency_vehicles:
                        self._test_route_optimization(emergency_vehicles)
                    
                    # Test traffic light control
                    # self._test_traffic_light_control(emergency_vehicles)
                    
                    # Test congestion prediction
                    # self._test_congestion_prediction()
                    
                    # Test V2X communication
                    # if emergency_vehicles:
                        # self._test_v2x_communication(emergency_vehicles)
                    
                    # Test metrics collection
                    # self._test_metrics_collection(step, emergency_vehicles)
                    
                    # Test visualization
                    # if step % 10 == 0:
                    #     self._test_visualization()
                    
                    step += 1
                    
                    # Print progress every 100 steps
                    if step % 100 == 0:
                        print(f"\n--- Simulation Progress: {step}/{max_steps} ---")
                        self._print_test_status()
                        
                except Exception as e:
                    print(f"\n❌ Error at step {step}:")
                    print(f"Error type: {type(e).__name__}")
                    print(f"Error message: {str(e)}")
                    break
                
        except Exception as e:
            print(f"\n❌ Fatal error during simulation:")
            print(f"Error type: {type(e).__name__}")
            print(f"Error message: {str(e)}")
        finally:
            print("\n=== Closing TraCI connection ===")
            traci.close()
            self._save_test_results()
    
    def _test_emergency_detection(self):
        """Test emergency vehicle detection"""
        try:
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
            
            if emergency_vehicles:
                self.test_results['emergency_detection'] = True
                print(f"✓ Detected {len(emergency_vehicles)} emergency vehicles")
            
            return emergency_vehicles
            
        except Exception as e:
            print(f"❌ Emergency detection error: {str(e)}")
            return []
    
    def _test_route_optimization(self, emergency_vehicles):
        """Test route optimization for emergency vehicles"""
        # try:
        for ev in emergency_vehicles:
            original_route = ev['route']
            optimized_route = self.router.optimize_route(ev)
            
            if optimized_route and optimized_route != original_route:
                self.test_results['route_optimization'] = True
                print(f"✓ Route optimized for {ev['id']}")
                    
        # except Exception as e:
        #     print(f"❌ Route optimization error: {str(e)}")
    
    def _test_traffic_light_control(self, emergency_vehicles):
        """Test traffic light control"""
        try:
            for tl_id in traci.trafficlight.getIDList():
                original_phase = traci.trafficlight.getPhase(tl_id)
                self.controller.update_traffic_lights(emergency_vehicles)
                new_phase = traci.trafficlight.getPhase(tl_id)
                
                if new_phase != original_phase:
                    self.test_results['traffic_light_control'] = True
                    print(f"✓ Traffic light {tl_id} phase changed: {original_phase} -> {new_phase}")
                    
        except Exception as e:
            print(f"❌ Traffic light control error: {str(e)}")
    
    def _test_congestion_prediction(self):
        """Test congestion prediction"""
        try:
            congestion_map = self.predictor.predict_congestion()
            if congestion_map:
                self.test_results['congestion_prediction'] = True
                print("✓ Congestion prediction active")
                
        except Exception as e:
            print(f"❌ Congestion prediction error: {str(e)}")
    
    def _test_v2x_communication(self, emergency_vehicles):
        """Test V2X communication"""
        try:
            self.v2x.process_communications(emergency_vehicles)
            self.test_results['v2x_communication'] = True
            print("✓ V2X communication active")
            
        except Exception as e:
            print(f"❌ V2X communication error: {str(e)}")
    
    def _test_metrics_collection(self, step, emergency_vehicles):
        """Test metrics collection"""
        try:
            self.metrics.update_metrics(step, emergency_vehicles)
            current_metrics = self.metrics.get_current_metrics()
            
            if current_metrics:
                self.test_results['metrics_collection'] = True
                print("✓ Metrics collection active")
                
        except Exception as e:
            print(f"❌ Metrics collection error: {str(e)}")
    
    def _test_visualization(self):
        """Test visualization"""
        try:
            current_metrics = self.metrics.get_current_metrics()
            self.visualizer.update_plots(current_metrics)
            self.test_results['visualization'] = True
            print("✓ Visualization active")
            
        except Exception as e:
            print(f"❌ Visualization error: {str(e)}")
    
    def _is_emergency_vehicle(self, veh_id):
        """Check if a vehicle is an emergency vehicle"""
        try:
            veh_type = traci.vehicle.getTypeID(veh_id)
            return veh_type in ['ambulance', 'fire_truck', 'police']
        except Exception as e:
            print(f"❌ Vehicle type check error for {veh_id}: {str(e)}")
            return False
    
    def _print_test_status(self):
        """Print current test status"""
        print("\nCurrent Test Status:")
        for test, status in self.test_results.items():
            print(f"{'✓' if status else '✗'} {test}")
    
    def _save_test_results(self):
        """Save test results to file"""
        print("\n=== Saving Test Results ===")
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            results_dir = project_root / 'test_results' / timestamp
            results_dir.mkdir(parents=True, exist_ok=True)
            
            # Save test results
            with open(results_dir / 'test_results.txt', 'w') as f:
                f.write("SEVPS System Test Results\n")
                f.write("=======================\n\n")
                for test, status in self.test_results.items():
                    f.write(f"{test}: {'PASS' if status else 'FAIL'}\n")
            
            # Save metrics
            self.metrics.save_metrics(results_dir)
            
            # Save visualizations
            self.visualizer.save_plots(results_dir)
            
            print(f"✓ Test results saved to: {results_dir}")
            
        except Exception as e:
            print(f"❌ Error saving test results: {str(e)}")

if __name__ == "__main__":
    # Path to your SUMO configuration file
    sumocfg_path = 'osm.sumocfg'
    
    # Initialize and run the tests
    tester = SEVPSTester(sumocfg_path)
    tester.run_tests() 