#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import math

class SimpleGpsRecorder(Node):
    def __init__(self):
        super().__init__('gps_recorder_node')

        self.create_subscription(Odometry, '/carla/ego_vehicle/odometry', self.listener_callback, 10)
        
        self.filename = 'recorded_path.csv'
        self.file = open(self.filename, 'w')
        self.writer = csv.writer(self.file)

        self.writer.writerow(['x', 'y'])
        
        self.last_x = 0.0
        self.last_y = 0.0
        self.first_run = True
        
        print(f"A rögzítés elindult! Fájl neve: {self.filename}")

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        

        if self.first_run:
            self.save_to_file(x, y)
            self.first_run = False
            return

        diff_x = x - self.last_x
        diff_y = y - self.last_y
        distance = math.sqrt(diff_x * diff_x + diff_y * diff_y)

        if distance > 1.0:
            self.save_to_file(x, y)

    def save_to_file(self, x, y):
        self.writer.writerow([x, y])
        

        self.last_x = x
        self.last_y = y
        
        print(f"Új pont mentve X: {x:.2f}, Y: {y:.2f}")

def main(args=None):
    rclpy.init(args=args)
    recorder = SimpleGpsRecorder()
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    
    recorder.file.close()
    recorder.destroy_node()
    rclpy.shutdown()
    print("\nA fájl sikeresen elmentve!")

if __name__ == '__main__':
    main()