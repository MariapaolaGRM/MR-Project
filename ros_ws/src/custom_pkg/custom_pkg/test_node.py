import rclpy
from rclpy.node import Node

import math
import time

from typing import Dict, Tuple
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from collections import Counter

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        
        # List of robots
        self.declare_parameter('robots', ['robot1','robot2'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value
        
        # Creating a dictionary for maps
        # maps = {rob1: {10: OccupancyGrid,
        #                  20: OG,...
        #                 },
        #          rob2: {10: OG,
        #                  20: OG,...
        #                 }}
        self.maps: Dict[str, Dict[int, Dict[str, int]]] = {robot: {} for robot in self.robot_names}

        # Dictionary to save the latest maps for each robot
        self.last_maps: Dict[str, Dict[str, int]] = {}


        # Dictionary for markers
        self.marker_dict = {}

        # Dictionary to save last markers
        self.last_marker: Dict[Tuple[str, int], Marker] = {}

        # Staring time
        self.start_time = self.get_clock().now()
        # Timer to store data every 10 seconds 
        self.timer = self.create_timer(10.0, self.timer_callback)

        # Marker topic subscription
        self.create_subscription(Marker, '/detected_objects_markers', self.marker_callback, 10)

        for robot in self.robot_names:
            # subscriber to robot/map topic
            topic = f'/{robot}/map'
            self.create_subscription(
                OccupancyGrid, topic, lambda msg, r=robot: self.map_callback(msg, r), 10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def map_callback(self, msg:OccupancyGrid, robot:str):
        '''Saves the last map of each robot'''
        data = msg.data # save only date values
        counts = Counter(data)
        # Types of cells
        unknown = counts.get(-1,0)
        free = counts.get(0,0)
        occupied = counts.get(100,0)
        total = len(data)

        self.last_maps[robot] = {
            'unknown' : unknown,
            'free' : free,
            'occupied' : occupied,
            'total' : total
        }

    def marker_callback(self, msg:Marker):
        '''Save the last markers'''
        key = (msg.ns, msg.id)

        now = self.get_clock().now()
        elapsed_time = int((now-self.start_time).nanoseconds*1e-9)
        # First detection 
        if key not in self.marker_dict:
            self.get_logger().info(f"Elapsed {elapsed_time} | header time {msg.header.stamp }")
            self.marker_dict[key] = {
                'first_seen' : msg.header.stamp,
                'positions': {}
            }
        self.last_marker[key] = msg

    def timer_callback(self):
        '''Save maps every 10 seconds'''
        now = self.get_clock().now()
        elapsed_time = int((now-self.start_time).nanoseconds*1e-9)

        for robot, data in self.last_maps.items():
            self.maps[robot][elapsed_time] = data.copy()
            explored = data['free']+data['occupied']
            percent = explored/data['total']*100
            self.get_logger().info(f"[{robot}] t = {elapsed_time}s | explored =  {percent: .2f}%") # percentage print
        
        self.get_logger().info(f"Cell dictionary {self.maps}")

        for key, marker in self.last_marker.items():
            pos = marker.pose.position
            self.marker_dict[key]['positions'][elapsed_time]=(
                pos.x, pos.y, pos.z
            )
        self.get_logger().info(f"Markers dictionary {self.marker_dict}")

def main(args=None):
    rclpy.init(args=args)
    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()
