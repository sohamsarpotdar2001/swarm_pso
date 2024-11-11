#!/usr/bin/env python3

import rospy
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Float64
import numpy as np

class LatencyMonitorGUI:
    def __init__(self, num_uavs):
        self.num_uavs = num_uavs
        
        # Initialize ROS node
        rospy.init_node('latency_monitor_gui', anonymous=True)
        self.rate = rospy.Rate(20)
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("UAV Latency Monitor")
        self.root.geometry("800x600")
        
        # Create a frame for the title
        title_frame = ttk.Frame(self.root)
        title_frame.pack(pady=10)
        ttk.Label(title_frame, text="UAV Latency Monitor", font=('Arial', 16, 'bold')).pack()
        
        # Create main frame for latency displays
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Configure grid columns
        self.main_frame.columnconfigure(0, weight=1)
        for i in range(num_uavs):
            self.main_frame.columnconfigure(i+1, weight=1)
        
        # Create headers
        ttk.Label(self.main_frame, text="Metric", font=('Arial', 12, 'bold')).grid(row=0, column=0, padx=5, pady=5, sticky='w')
        for i in range(num_uavs):
            ttk.Label(self.main_frame, text=f"UAV {i}", font=('Arial', 12, 'bold')).grid(row=0, column=i+1, padx=5, pady=5)
        
        # Create labels for different latency metrics
        self.metrics = ['Processing', 'Transmission', 'Queuing']
        self.latency_labels = {}
        
        for i, metric in enumerate(self.metrics, 1):
            ttk.Label(self.main_frame, text=f"{metric} Latency (ms):", font=('Arial', 10)).grid(row=i, column=0, padx=5, pady=5, sticky='w')
            self.latency_labels[metric] = []
            
            for j in range(num_uavs):
                label = ttk.Label(self.main_frame, text="0.000", font=('Arial', 10))
                label.grid(row=i, column=j+1, padx=5, pady=5)
                self.latency_labels[metric].append(label)
        
        # Create statistics frame
        stats_frame = ttk.LabelFrame(self.root, text="Statistics", padding=10)
        stats_frame.pack(fill=tk.X, padx=20, pady=10)
        
        self.stats_labels = {}
        for i, metric in enumerate(self.metrics):
            ttk.Label(stats_frame, text=f"Avg {metric}:").grid(row=i//2, column=i%2*2, padx=5, pady=2, sticky='w')
            self.stats_labels[metric] = ttk.Label(stats_frame, text="0.000 ms")
            self.stats_labels[metric].grid(row=i//2, column=i%2*2+1, padx=5, pady=2, sticky='w')
        
        # Initialize data storage for statistics
        self.latency_history = {metric: [[] for _ in range(num_uavs)] for metric in self.metrics}
        
        # Subscribe to latency topics - Using separate callback functions
        self.subscribers = []
        for i in range(num_uavs):
            self.subscribers.append(rospy.Subscriber(
                f"/uav{i}/processing_latency",
                Float64,
                self.create_callback("Processing", i)
            ))
            self.subscribers.append(rospy.Subscriber(
                f"/uav{i}/transmission_latency",
                Float64,
                self.create_callback("Transmission", i)
            ))
            self.subscribers.append(rospy.Subscriber(
                f"/uavs/queuing_latency",
                Float64,
                self.create_callback('Queuing', i)
            ))

    def create_callback(self, metric, uav_id):
        """Create a callback function for a specific metric and UAV"""
        def callback(msg):
            try:
                latency_ms = abs(float(msg.data) / 1000000)  # Convert to milliseconds
                if not np.isnan(latency_ms):
                    self.latency_labels[metric][uav_id].configure(text=f"{latency_ms:4.2f}")
                    self.latency_history[metric][uav_id].append(latency_ms)
                    print(self.latency_history)
                    
                    # Keep only last 100 values
                    if len(self.latency_history[metric][uav_id]) > 100:
                        self.latency_history[metric][uav_id].pop(0)
            except (IndexError, ValueError) as e:
                rospy.logwarn(f"Error processing latency data for UAV {uav_id}, metric {metric}: {e}")
        return callback
    
    def update_statistics(self):
        """Update average statistics for each metric"""
        try:
            for metric in self.metrics:
                # Calculate average across all UAVs and their history
                all_values = []
                for uav_history in self.latency_history[metric]:
                    if uav_history:  # Only include non-empty histories
                        all_values.extend(uav_history)
                
                if all_values:
                    avg = np.mean(all_values)
                    self.stats_labels[metric].configure(text=f"{avg:4.2f} ms")
            
            # Schedule next update
            self.root.after(1000, self.update_statistics)
        except Exception as e:
            rospy.logwarn(f"Error updating statistics: {e}")
            self.root.after(1000, self.update_statistics)
    
    def run(self):
        """Start the GUI main loop"""
        try:
            # Start the statistics update
            self.root.after(1000, self.update_statistics)
            self.root.mainloop()
        except rospy.ROSInterruptException:
            pass
        finally:
            rospy.signal_shutdown("GUI closed")

if __name__ == "__main__":
    try:
        gui = LatencyMonitorGUI(num_uavs=10)
        gui.run()
    except rospy.ROSInterruptException:
        pass