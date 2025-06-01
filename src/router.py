import traci
import networkx as nx
import numpy as np
from typing import Dict, List, Optional, Tuple
import sumolib
from pathlib import Path

class EmergencyRouter:
    def __init__(self):
        """Initialize the emergency router with a directed graph"""
        self.network = nx.DiGraph()
        # Load the network file
        self.net = sumolib.net.readNet('additional/osm.net.xml')
        # Network will be built when first needed
        self.nodes = [node.getID() for node in self.net.getNodes()]
    
    def _build_network(self):
        """Build the network graph from SUMO network"""
        # Clear existing network
        self.network.clear()
        # ===============================================
        # Add nodes (junctions)
        for junction in self.net.getNodes():
            self.network.add_node(junction.getID())
        # ===============================================

        # Add edges (roads)
        for edge in self.net.getEdges():
            from_node = edge.getFromNode().getID()
            to_node = edge.getToNode().getID()
            length = edge.getLength()
            self.network.add_edge(from_node, to_node, weight=length, edge_id=edge.getID())

        print ("Loaded graph with ", len(self.network.nodes()), " nodes and ", len(self.network.edges()), " edges")

        # ===============================================

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
        if current_edge.startswith(":"):
            print ("==== [Debug] ==== : Vehicle is in junction")
            parts = current_edge.replace(":", "").split("_")[:-1]
            current_node = "_".join(parts)
            return None
        elif current_edge in self.nodes:
            print ("==== [Debug] ==== : Vehicle is in junction")
            current_node = current_edge
            return None
        else:   
            current_node = self._get_current_node(current_edge,type='from')

        print ("==== [Debug] ==== : current_edge: ",current_edge)

        print ("==== [Debug] ==== : current_node: ",current_node)

        route = emergency_vehicle['route']
        destination = route[-1]
        destination_node = self._get_current_node(destination,type='to')
        print ("==== [Debug] ==== : destination_edge: ",destination)
        print ("==== [Debug] ==== : destination_node: ",destination_node)

        if current_node == destination_node:
            print ("==== [Debug] ==== : current_node is the same as destination_node")
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
            print ("==== [Debug] ==== : edge_path: ",path)
            return [current_edge] + edge_path
            
        except nx.NetworkXNoPath:
            print(f"No path found for vehicle {emergency_vehicle['id']}")
            return None
    
    def _update_edge_weights(self):
        """Update edge weights based on current traffic conditions"""

        edge_id_list = [edge.getID() for edge in self.net.getEdges()]

        
        # for edge_id in traci.edge.getIDList():
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
    def _get_current_node(self, edge_id,type='from'):
        edge = self.net.getEdge(edge_id)
        if edge:
            if type == 'from':
                return edge.getToNode().getID()
            elif type == 'to':
                return edge.getToNode().getID()
        else:
            return None
        
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