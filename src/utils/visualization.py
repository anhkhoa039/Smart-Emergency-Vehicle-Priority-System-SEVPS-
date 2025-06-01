import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import numpy as np
from typing import Dict
import os

class Visualizer:
    def __init__(self):
        self.fig = None
        self.plotly_fig = None
        self.metrics_history = {
            'emergency_response_times': [],
            'traffic_light_efficiencies': [],
            'average_speeds': [],
            'waiting_times': []
        }
        
    def update_plots(self, current_metrics: Dict):
        """Update all visualization plots with current metrics"""
        self._update_metrics_history(current_metrics)
        self._update_matplotlib_plots()
        self._update_plotly_plots()
    
    def _update_metrics_history(self, current_metrics: Dict):
        """Update the metrics history with current data"""
        # Update emergency vehicle metrics
        ev_metrics = current_metrics['emergency_vehicles']
        if ev_metrics:
            response_times = [m['time_loss'] for m in ev_metrics.values()]
            self.metrics_history['emergency_response_times'].append(np.mean(response_times))
        
        # Update traffic light metrics
        tl_metrics = current_metrics['traffic_lights']
        if tl_metrics:
            efficiencies = []
            for tl in tl_metrics.values():
                waiting = tl['waiting_vehicles']
                total = len(tl['controlled_lanes'])
                if total > 0:
                    efficiencies.append(1 - (waiting / total))
            self.metrics_history['traffic_light_efficiencies'].append(np.mean(efficiencies))
        
        # Update general traffic metrics
        traffic_metrics = current_metrics['general_traffic']
        self.metrics_history['average_speeds'].append(traffic_metrics['average_speed'])
        self.metrics_history['waiting_times'].append(traffic_metrics['total_waiting_time'])
    
    def _update_matplotlib_plots(self):
        """Update matplotlib plots"""
        if self.fig is None:
            self.fig, self.axes = plt.subplots(2, 2, figsize=(15, 10))
            self.fig.suptitle('SEVPS Real-time Metrics')
        
        # Clear previous plots
        for ax in self.axes.flat:
            ax.clear()
        
        # Plot emergency response times
        self.axes[0, 0].plot(self.metrics_history['emergency_response_times'])
        self.axes[0, 0].set_title('Emergency Vehicle Response Times')
        self.axes[0, 0].set_xlabel('Simulation Step')
        self.axes[0, 0].set_ylabel('Time Loss (s)')
        
        # Plot traffic light efficiency
        self.axes[0, 1].plot(self.metrics_history['traffic_light_efficiencies'])
        self.axes[0, 1].set_title('Traffic Light Efficiency')
        self.axes[0, 1].set_xlabel('Simulation Step')
        self.axes[0, 1].set_ylabel('Efficiency')
        
        # Plot average speeds
        self.axes[1, 0].plot(self.metrics_history['average_speeds'])
        self.axes[1, 0].set_title('Average Vehicle Speed')
        self.axes[1, 0].set_xlabel('Simulation Step')
        self.axes[1, 0].set_ylabel('Speed (m/s)')
        
        # Plot waiting times
        self.axes[1, 1].plot(self.metrics_history['waiting_times'])
        self.axes[1, 1].set_title('Total Waiting Time')
        self.axes[1, 1].set_xlabel('Simulation Step')
        self.axes[1, 1].set_ylabel('Time (s)')
        
        plt.tight_layout()
        plt.pause(0.01)
    
    def _update_plotly_plots(self):
        """Update Plotly interactive plots"""
        if self.plotly_fig is None:
            self.plotly_fig = make_subplots(
                rows=2, cols=2,
                subplot_titles=(
                    'Emergency Vehicle Response Times',
                    'Traffic Light Efficiency',
                    'Average Vehicle Speed',
                    'Total Waiting Time'
                )
            )
        
        # Update traces
        self.plotly_fig.data = []  # Clear previous traces
        
        # Add emergency response times
        self.plotly_fig.add_trace(
            go.Scatter(
                y=self.metrics_history['emergency_response_times'],
                name='Response Time'
            ),
            row=1, col=1
        )
        
        # Add traffic light efficiency
        self.plotly_fig.add_trace(
            go.Scatter(
                y=self.metrics_history['traffic_light_efficiencies'],
                name='Efficiency'
            ),
            row=1, col=2
        )
        
        # Add average speeds
        self.plotly_fig.add_trace(
            go.Scatter(
                y=self.metrics_history['average_speeds'],
                name='Speed'
            ),
            row=2, col=1
        )
        
        # Add waiting times
        self.plotly_fig.add_trace(
            go.Scatter(
                y=self.metrics_history['waiting_times'],
                name='Waiting Time'
            ),
            row=2, col=2
        )
        
        # Update layout
        self.plotly_fig.update_layout(
            height=800,
            width=1200,
            title_text='SEVPS Real-time Metrics',
            showlegend=True
        )
    
    def save_plots(self, directory: str):
        """Save all plots to files"""
        # Create directory if it doesn't exist
        os.makedirs(directory, exist_ok=True)
        
        # Save matplotlib figure
        if self.fig is not None:
            self.fig.savefig(f"{directory}/metrics_plots.png")
        
        # Save Plotly figure
        if self.plotly_fig is not None:
            self.plotly_fig.write_html(f"{directory}/metrics_plots.html")
    
    def show_plots(self):
        """Display the plots"""
        if self.fig is not None:
            plt.show()
        
        if self.plotly_fig is not None:
            self.plotly_fig.show() 