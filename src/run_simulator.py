import os
import sys
import traci
import sumolib
import time
from pathlib import Path
from router import EmergencyRouter

def highlight_emergency_vehicle(vehicle_id):
    """Highlight emergency vehicle in the simulation"""
    # Set vehicle color to red
    traci.vehicle.setColor(vehicle_id, (255, 0, 0, 255))
    # Set vehicle width to be more visible
    traci.vehicle.setWidth(vehicle_id, 2.0)
    # Add a blinking effect
    traci.vehicle.setMinGap(vehicle_id, 2.0)

def announce_emergency(vehicle_id, vehicle_type):
    """Announce emergency vehicle presence"""
    print("\n" + "!"*50)
    print(f"🚨 EMERGENCY VEHICLE DETECTED 🚨")
    print(f"Type: {vehicle_type}")
    print(f"ID: {vehicle_id}")
    print(f"Current Position: {traci.vehicle.getRoadID(vehicle_id)}")
    print("!"*50 + "\n")

def update_vehicle_route(vehicle_id, new_route):
    """Update vehicle's route with the optimized route"""
    try:
        # Get current route
        current_route = traci.vehicle.getRoute(vehicle_id)
        
        # Only update if the new route is different
        if new_route != current_route:
            print(f"\n🔄 Updating route for vehicle {vehicle_id}")
            print(f"Original route: {current_route}")
            print(f"New optimized route: {new_route}")
            
            # Update the route
            traci.vehicle.setRoute(vehicle_id, new_route)
            print("✓ Route updated successfully")
            
            # Highlight the new route edges
            for edge_id in new_route:
                traci.edge.setColor(edge_id, (0, 255, 0, 255))  # Green color for optimized route
    except Exception as e:
        print(f"❌ Error updating route: {str(e)}")

def run_simulation():
    """Run SUMO simulation with emergency vehicle monitoring"""
    try:
        # Initialize SUMO
        print("=== Starting SUMO Simulation ===")
        
        # Get the path to the SUMO configuration file
        sumocfg_path = 'osm.sumocfg'
        
        # Start SUMO with GUI
        sumo_binary = sumolib.checkBinary('sumo-gui')
        traci.start([sumo_binary, '-c', sumocfg_path])
        
        # Initialize router
        router = EmergencyRouter()
        
        # Simulation loop
        step = 0
        max_steps = 3600  # Run for 1 hour of simulation time
        emergency_vehicles_seen = set()  # Track emergency vehicles we've already announced
        optimized_vehicles = set()  # Track vehicles that have been optimized
        
        print(f"Simulation will run for {max_steps} steps")
        
        while step < max_steps and traci.simulation.getMinExpectedNumber() > 0:
            # Perform simulation step
            traci.simulationStep()
            
            # Check for emergency vehicles
            for veh_id in traci.vehicle.getIDList():
                veh_type = traci.vehicle.getTypeID(veh_id)
                
                # Check if it's an emergency vehicle
                if veh_type in ['ambulance', 'fire_truck', 'police']:
                    # If we haven't seen this vehicle before, announce it
                    if veh_id not in emergency_vehicles_seen:
                        emergency_vehicles_seen.add(veh_id)
                        announce_emergency(veh_id, veh_type)
                        highlight_emergency_vehicle(veh_id)
                    
                    # Get current position and speed
                    current_edge = traci.vehicle.getRoadID(veh_id)
                    speed = traci.vehicle.getSpeed(veh_id)
                    position = traci.vehicle.getPosition(veh_id)
                    
                    # Try to optimize route if not already optimized
                    if veh_id not in optimized_vehicles:
                        current_route = traci.vehicle.getRoute(veh_id)
                        emergency_info = {
                            'id': veh_id,
                            'type': veh_type,
                            'route': current_route
                        }
                        
                        optimized_route = router.optimize_route(emergency_info)
                        if optimized_route and optimized_route != current_route:
                            update_vehicle_route(veh_id, optimized_route)
                            optimized_vehicles.add(veh_id)
                    
                    # Print status every 100 steps
                    if step % 100 == 0:
                        print(f"Emergency Vehicle {veh_id}:")
                        print(f"  - Current Edge: {current_edge}")
                        print(f"  - Speed: {speed:.2f} m/s")
                        print(f"  - Position: ({position[0]:.2f}, {position[1]:.2f})")
                        print(f"  - Route Optimized: {'Yes' if veh_id in optimized_vehicles else 'No'}")
            
            # Print simulation progress every 100 steps
            if step % 100 == 0:
                print(f"\n--- Step {step} ---")
                print(f"Total vehicles: {len(traci.vehicle.getIDList())}")
                print(f"Emergency vehicles: {len(emergency_vehicles_seen)}")
                print(f"Optimized routes: {len(optimized_vehicles)}")
            
            step += 1
            time.sleep(0.1)  # Add a small delay to make the simulation visible
        
        print("\n=== Simulation Complete ===")
        print(f"Total steps: {step}")
        print(f"Total emergency vehicles encountered: {len(emergency_vehicles_seen)}")
        print(f"Total routes optimized: {len(optimized_vehicles)}")
        
    except Exception as e:
        print(f"\n❌ Error during simulation: {str(e)}")
    finally:
        # Close TraCI connection
        traci.close()

if __name__ == "__main__":
    run_simulation() 