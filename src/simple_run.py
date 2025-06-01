import os
import sys
import traci
import sumolib
import networkx as nx


def run_simulation():
    """Run a simple SUMO simulation with TraCI"""
    try:
        # Initialize SUMO
        print("=== Starting SUMO Simulation ===")
        
        # Get the path to the SUMO configuration file
        sumocfg_path = 'osm.sumocfg'
        
        # Start SUMO with GUI
        sumo_binary = sumolib.checkBinary('sumo')
        traci.start([sumo_binary, '-c', sumocfg_path])
        
        # Simulation loop
        step = 0
        max_steps = 500  # Run for 1 hour of simulation time
        
        print(f"Simulation will run for {max_steps} steps")
        
        net = sumolib.net.readNet('osm.net.xml')
        graph = nx.DiGraph()
        # Add nodes (junctions)
        for junction in net.getNodes():
            graph.add_node(junction.getID())

        # Add edges (roads)
        for edge in net.getEdges():
            from_node = edge.getFromNode().getID()
            to_node = edge.getToNode().getID()
            length = edge.getLength()
            graph.add_edge(from_node, to_node, weight=length, edge_id=edge.getID())

        print ("loaded graph with ", len(graph.nodes()), " nodes and ", len(graph.edges()), " edges")

        while step < max_steps and traci.simulation.getMinExpectedNumber() > 0:
            # Perform simulation step
            traci.simulationStep()
            #===============================================
    
      


            #===============================================
            # # Get all edges
            # edges = traci.edge.getIDList()
            # print("Number of edges: ", len(edges))

            # print (edges[:5])
            # edge_id = edges[0]

            # from_node = traci.edge.getFromNode(edge_id)
            # to_node = traci.edge.getToNode(edge_id)

            # print(f"Edge {edge_id} starts at {from_node} and ends at {to_node}")


        

            # Get the first edge
            # first_edge = edges[0]
            # print("First edge: ", first_edge)
            # # Get the number of lanes for the first edge
            # first_edge_lanes = traci.edge.getLaneNumber(first_edge)
            # print("First edge lanes: ", first_edge_lanes)
            # lane_0 = f"{first_edge}_0"
            # print("Lane 0: ", lane_0)
            # lane_0_length = traci.lane.getLength(lane_0)
            # print("Lane 0 length: ", lane_0_length)
       
            # first_edge_lane = first_edge_lanes[0]
            # print("First edge lane: ", first_edge_lane)
            # first_edge_lane_length = traci.lane.getLength(first_edge_lane)

            

            # Print some basic information every 100 steps
            if step % 100 == 0:
                print(f"\n--- Step {step} ---")
                
                # Get number of vehicles
                vehicle_count = len(traci.vehicle.getIDList())
                print(f"Vehicles in simulation: {vehicle_count}")
                
                # Get number of traffic lights
                tl_count = len(traci.trafficlight.getIDList())
                print(f"Traffic lights: {tl_count}")
                
                # Check for emergency vehicles
                emergency_vehicles = [veh_id for veh_id in traci.vehicle.getIDList() 
                                   if traci.vehicle.getTypeID(veh_id) in ['ambulance', 'fire_truck', 'police']]
                if emergency_vehicles:
                    print(f"Emergency vehicles: {len(emergency_vehicles)}")
                    for ev in emergency_vehicles:
                        print(f"  - {ev}: {traci.vehicle.getTypeID(ev)}")
            
            step += 1
        
        print("\n=== Simulation Complete ===")
        print(f"Total steps: {step}")
        
    except Exception as e:
        print(f"\n❌ Error during simulation: {str(e)}")
    finally:
        # Close TraCI connection
        traci.close()

if __name__ == "__main__":
    run_simulation() 