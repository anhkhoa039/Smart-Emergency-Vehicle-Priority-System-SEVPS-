import traci
import numpy as np
import tensorflow as tf
from typing import Dict, List, Optional
import pandas as pd
from sklearn.preprocessing import MinMaxScaler

class TrafficLightController:
    def __init__(self):
        """Initialize the traffic light controller"""
        self.model = self._build_model()
        self.scaler = MinMaxScaler()
        self.state_size = 3  # [current_phase, incoming_vehicles, emergency_presence]
    
    def _build_model(self):
        """Build the neural network model for traffic light control"""
        model = tf.keras.Sequential([
            tf.keras.layers.Dense(64, activation='relu', input_shape=(3,)),
            tf.keras.layers.Dense(32, activation='relu'),
            tf.keras.layers.Dense(1, activation='sigmoid')
        ])
        model.compile(optimizer='adam', loss='mse')
        return model
    
    def update_traffic_lights(self, emergency_vehicles: List[Dict]):
        """Update traffic light phases based on current traffic conditions"""
        try:
            for tl_id in traci.trafficlight.getIDList():
                # Get current state
                state = self._get_state(tl_id, emergency_vehicles)
                
                # Make prediction
                state = np.array(state).reshape(1, -1)
                state = self.scaler.fit_transform(state)
                action = self.model.predict(state, verbose=0)[0][0]
                
                # Apply action (0: keep current phase, 1: switch to next phase)
                if action > 0.5:
                    current_phase = traci.trafficlight.getPhase(tl_id)
                    traci.trafficlight.setPhase(tl_id, (current_phase + 1) % traci.trafficlight.getPhaseNumber(tl_id))
                
        except Exception as e:
            print(f"❌ Traffic light control error: {str(e)}")
    
    def _get_state(self, tl_id: str, emergency_vehicles: List[Dict]) -> List[float]:
        """Get current state for a traffic light"""
        try:
            # Get current phase
            current_phase = traci.trafficlight.getPhase(tl_id)
            
            # Get incoming vehicles
            incoming_vehicles = 0
            for edge_id in traci.trafficlight.getControlledLinks(tl_id):
                incoming_vehicles += traci.edge.getLastStepVehicleNumber(edge_id[0])
            
            # Check for emergency vehicles
            emergency_presence = 0
            if emergency_vehicles:
                for ev in emergency_vehicles:
                    if self._is_emergency_near_traffic_light(ev, tl_id):
                        emergency_presence = 1
                        break
            
            return [
                current_phase / traci.trafficlight.getPhaseNumber(tl_id),  # Normalized phase
                incoming_vehicles / 10.0,  # Normalized vehicle count
                emergency_presence  # Binary emergency presence
            ]
            
        except Exception as e:
            print(f"❌ Error getting state for {tl_id}: {str(e)}")
            return [0, 0, 0]
    
    def _is_emergency_near_traffic_light(self, emergency_vehicle: Dict, tl_id: str) -> bool:
        """Check if an emergency vehicle is near a traffic light"""
        try:
            # Get traffic light position
            tl_pos = traci.junction.getPosition(tl_id)
            
            # Get emergency vehicle position
            ev_pos = emergency_vehicle['position']
            
            # Calculate distance
            distance = np.sqrt((tl_pos[0] - ev_pos[0])**2 + (tl_pos[1] - ev_pos[1])**2)
            
            # Return True if within 100 meters
            return distance < 100
            
        except Exception as e:
            print(f"❌ Error checking emergency vehicle proximity: {str(e)}")
            return False
    
    def train(self, historical_data: pd.DataFrame):
        """Train the model using historical data"""
        try:
            # Prepare training data
            X = historical_data[['phase', 'incoming_vehicles', 'emergency_presence']].values
            y = historical_data['action'].values
            
            # Normalize features
            X = self.scaler.fit_transform(X)
            
            # Train model
            self.model.fit(X, y, epochs=10, batch_size=32, verbose=0)
            
        except Exception as e:
            print(f"❌ Error training model: {str(e)}")
    
    def save_model(self, path: str):
        """Save the trained model"""
        try:
            self.model.save(path)
        except Exception as e:
            print(f"❌ Error saving model: {str(e)}")
    
    def load_model(self, path: str):
        """Load a trained model"""
        try:
            self.model = tf.keras.models.load_model(path)
        except Exception as e:
            print(f"❌ Error loading model: {str(e)}") 