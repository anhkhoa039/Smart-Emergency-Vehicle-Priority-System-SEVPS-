import traci
import numpy as np
import tensorflow as tf
from typing import Dict, List, Optional
import pandas as pd
from sklearn.preprocessing import MinMaxScaler

class CongestionPredictor:
    def __init__(self, sequence_length=10):
        """Initialize the congestion predictor"""
        self.sequence_length = sequence_length
        self.history = {}  # Store historical data for each edge
        self.model = self._build_model()
        self.scaler = MinMaxScaler()
    
    def _build_model(self):
        """Build the LSTM model for congestion prediction"""
        model = tf.keras.Sequential([
            tf.keras.layers.LSTM(64, input_shape=(self.sequence_length, 3), return_sequences=True),
            tf.keras.layers.LSTM(32),
            tf.keras.layers.Dense(1, activation='sigmoid')
        ])
        model.compile(optimizer='adam', loss='mse')
        return model
    
    def predict_congestion(self) -> Dict[str, float]:
        """Predict congestion levels for all edges"""
        congestion_map = {}
        
        try:
            # Get all edges
            edges = traci.edge.getIDList()
            
            for edge_id in edges:
                # Get current data
                current_data = self._get_edge_data(edge_id)
                
                # Add to history
                if edge_id not in self.history:
                    self.history[edge_id] = []
                self.history[edge_id].append(current_data)
                
                # Keep only recent history
                if len(self.history[edge_id]) > self.sequence_length:
                    self.history[edge_id] = self.history[edge_id][-self.sequence_length:]
                
                # Make prediction if we have enough history
                if len(self.history[edge_id]) == self.sequence_length:
                    # Prepare sequence for prediction
                    sequence = np.array(self.history[edge_id])
                    sequence = self.scaler.fit_transform(sequence)
                    sequence = np.reshape(sequence, (1, self.sequence_length, 3))
                    
                    # Make prediction
                    prediction = self.model.predict(sequence, verbose=0)[0][0]
                    congestion_map[edge_id] = float(prediction)
                else:
                    # Use normalized vehicle count as prediction if not enough history
                    congestion_map[edge_id] = current_data[0] / 10.0  # Normalize vehicle count
            
            return congestion_map
            
        except Exception as e:
            print(f"Error in congestion prediction: {str(e)}")
            return {}
    
    def _get_edge_data(self, edge_id: str) -> List[float]:
        """Get current traffic data for an edge"""
        try:
            # Get number of lanes and construct lane IDs
            num_lanes = traci.edge.getLaneNumber(edge_id)
            lanes = [f"{edge_id}_{i}" for i in range(num_lanes)]
            
            # Calculate average metrics across all lanes
            total_vehicles = 0
            total_speed = 0
            total_waiting = 0
            
            for lane in lanes:
                # Get vehicle count
                vehicles = traci.lane.getLastStepVehicleNumber(lane)
                total_vehicles += vehicles
                
                # Get mean speed
                speed = traci.lane.getLastStepMeanSpeed(lane)
                total_speed += speed
                
                # Get waiting vehicles
                waiting = traci.lane.getLastStepHaltingNumber(lane)
                total_waiting += waiting
            
            # Calculate averages
            if num_lanes > 0:
                return [
                    total_vehicles / num_lanes,  # average vehicle count
                    total_speed / num_lanes,     # average speed
                    total_waiting / num_lanes    # average waiting vehicles
                ]
            else:
                return [0, 0, 0]
                
        except Exception as e:
            print(f"Error getting edge data for {edge_id}: {str(e)}")
            return [0, 0, 0]
    
    def train(self, historical_data: pd.DataFrame):
        """Train the model using historical data"""
        try:
            # Prepare sequences
            X, y = self._prepare_sequences(historical_data)
            
            # Normalize data
            X = self.scaler.fit_transform(X.reshape(-1, X.shape[-1])).reshape(X.shape)
            
            # Train model
            self.model.fit(X, y, epochs=10, batch_size=32, verbose=0)
            
        except Exception as e:
            print(f"Error training model: {str(e)}")
    
    def _prepare_sequences(self, data: pd.DataFrame) -> tuple:
        """Prepare input sequences and targets for LSTM training"""
        sequences = []
        targets = []
        
        for edge_id in data['edge_id'].unique():
            edge_data = data[data['edge_id'] == edge_id]
            
            for i in range(len(edge_data) - self.sequence_length):
                sequence = edge_data.iloc[i:i + self.sequence_length][['vehicles', 'speed', 'waiting']].values
                target = edge_data.iloc[i + self.sequence_length]['vehicles']  # Use vehicle count as target
                sequences.append(sequence)
                targets.append(target)
        
        return np.array(sequences), np.array(targets)
    
    def save_model(self, path: str):
        """Save the trained model"""
        try:
            self.model.save(path)
        except Exception as e:
            print(f"Error saving model: {str(e)}")
    
    def load_model(self, path: str):
        """Load a trained model"""
        try:
            self.model = tf.keras.models.load_model(path)
        except Exception as e:
            print(f"Error loading model: {str(e)}")
    
    def get_current_congestion(self, edge_id: str) -> float:
        """Get current congestion level for an edge"""
        try:
            data = self._get_edge_data(edge_id)
            return data[0] / 10.0  # Return normalized vehicle count as congestion measure
        except Exception as e:
            print(f"Error getting congestion for {edge_id}: {str(e)}")
            return 0.0 