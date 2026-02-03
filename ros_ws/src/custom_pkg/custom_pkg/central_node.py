import rclpy
from rclpy.node import Node

import math
import time
from custom_msg.msg import Box
from typing import Dict, Tuple

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool

class CentralNode(Node):
    def __init__(self):
        super().__init__('central_node')
        self.start_time = self.get_clock().now()
        
        self.declare_parameter('robots', ['robot1','robot2'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value 

        # Subscriber on /yolo/detections for each robot
        for robot in self.robot_names:
            topic = f'/{robot}/yolo/detections'
            self.create_subscription(
                Box, topic, lambda msg, r=robot: self.box_callback(msg, r), 10
            )
            self.get_logger().info(f"Subscribed to {topic}") 

        # data structure (dictionary): objects[classe][id] -> {"avg": (x,y,z), "positions": [(x,y,z),...], "n": int}
        self.objects: Dict[int, Dict[str, Dict]] = {}
        self.id = 0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initial robot positions
        self.robot_positions = [[0.0, -1.0], [0.0, 1.0]]
        self.robots_sent_home = False
        self.count = 0
        
        # Publisher Rviz Marker 
        self.marker_pub = self.create_publisher(Marker, '/detected_objects_markers', 10)
        self.create_timer(0.5, self.publish_markers)  

        # Action to return robots to a selected position 
        self.nav_clients = {} 
        self.resume_pub = {}
        self.goal_node_control_pub = {}

        for robot in self.robot_names:
            topic = f'/{robot}/navigate_to_pose'
            self.nav_clients[robot] = ActionClient(
                self,
                NavigateToPose,
                topic
            )
            self.get_logger().info(f"Created ActionClient for {robot} on {topic}")
        
            # Explorer resume publisher 
            self.resume_pub[robot] = self.create_publisher(
                Bool,
                f"/{robot}/explore/resume",
                10
            )

            # Publisher to disable goal node
            self.goal_node_control_pub[robot] = self.create_publisher(
                Bool,
                f"/{robot}/goal_node/active",
                10
            )
    
    def pixel_to_meters(self, width: float, height: float, depth_m: float) -> Tuple[float,float,float]:
        '''Convert (u,v,depth) to camera coordinates using pinhole model'''

        # Camera parameters: fx, fy, cx, cy (obtained from the camera_info topic)
        fx = 320.25492609007654
        fy = 320.25492609007654        
        
        # Calculation of the 3D position of the object in the camera reference system
        x = width * depth_m / fx
        y = height * depth_m / fy
        return (x, y)
    
    def compute_average(self, pmap: Tuple[float,float,float], positions: Tuple[float,float,float]) -> Tuple[float,float,float]:
        '''Function that calculates the moving average, giving more weight to previous measurements'''
        alpha = 0.6 # confidence for the weighted average
        
        sx = positions[0]*alpha + pmap[0]*(1-alpha) 
        sy = positions[1]*alpha + pmap[1]*(1-alpha)
        sz = positions[2]*alpha + pmap[2]*(1-alpha)
        return (sx, sy, sz)
    
    def send_nav_goal(self, robot, x, y): 
        '''Function that create goals to send'''
        client = self.nav_clients[robot]

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y

        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        goal.pose = pose

        client.wait_for_server()
        client.send_goal_async(goal)

        self.get_logger().info(f"Goal sent to {robot} at x={x}, y={y}")

    def send_robots_home(self):
        """Send the robots to their ending positions"""
        for i, robot in enumerate(self.robot_names):
            x, y = self.robot_positions[i]
            msg = Bool()
            msg.data = False
            # Stop explore_node
            self.resume_pub[robot].publish(msg)
            self.send_nav_goal(robot, x, y)
            self.get_logger().info(f"{robot} inviato a ({x}, {y})")
            # Stop goal_node
            self.goal_node_control_pub[robot].publish(msg)

    def check_all_objects_found(self) -> bool:
        """Check whether all objects have been found with sufficient confidence."""
        self.count_0 = 0
        self.count_1 = 0
        self.count_2 = 0
        if len(self.objects) != 3:
            return False
        
        for classe in [0, 1, 2]:
            #self.get_logger().info(f"Log 1 {classe}")
            if classe not in self.objects: # Check if the current class exists in the self.objects dictionary
                #self.get_logger().info(f"Log 2")
                return False
            if len(self.objects[classe]) < 2: # Check whether exactly 1 object was found for the current class
                #self.get_logger().info(f"Log 3 {len(self.objects[classe])}")
                return False
            else: 
                #self.get_logger().info(f"oggetti della classe {classe}: {len(self.objects[classe])}")
                for obj_id, obj in self.objects[classe].items():
                    if obj['n'] > 110:
                        if classe == 0:
                            self.count_0 += 1
                            self.get_logger().info(f"n = {obj['n']}, object = {classe}, count = {self.count_0}")
                        elif classe == 1:
                            self.count_1 += 1
                            self.get_logger().info(f"n = {obj['n']}, object = {classe}, count = {self.count_1}")
                        elif classe == 2:
                            self.count_2 += 1    
                            self.get_logger().info(f"n = {obj['n']}, object = {classe}, count = {self.count_2}")
        
        # return self.count_0 == 3 and self.count_1 == 3 and self.count_2 == 2 # For house 2 and 5
        return self.count_0 == 2 and self.count_1 == 2 and self.count_2 == 2 # For house 1, 3 and 4
    
    def box_callback(self, msg: Box, robot: str):
        try:
            # Reading values from the message
            src_frame = msg.header.frame_id
            classe = int(msg.classe)
            xc = float(msg.xc) 
            yc = float(msg.yc) 
            w = float(msg.w)
            h = float(msg.h)
            depth_m = float(msg.distance) # Distance from the object in the robot frame

            # Valid depth check 
            if depth_m <= 0.0 or math.isinf(depth_m) or math.isnan(depth_m):
                # self.get_logger().warn(f"[{robot}] depth not valid: {depth_m}")
                return

            # Discard boxes larger than 0.8x1.0 m
            box_width_m, box_height_m = self.pixel_to_meters(w, h, depth_m)
            if box_width_m > 0.8 or box_height_m > 1.0:
                return

            # Calculation of the 3D position of the object in the camera reference system
            cx = 320.0
            cy = 240.0 
            cam_x, cam_y = self.pixel_to_meters(xc-cx, yc-cy, depth_m)
            cam_z = depth_m

            # Listener for the transformation between object coordinates in the optical frame and coordinates in the map frame
            p_cam = PointStamped() 
            p_cam.header.frame_id = src_frame
            p_cam.point.x = cam_x
            p_cam.point.y = cam_y
            p_cam.point.z = cam_z

            try:
                global_frame = 'map'
                trans = self.tf_buffer.lookup_transform(global_frame, src_frame,rclpy.time.Time())
                p_map = do_transform_point(p_cam, trans) # apply the trans transformation to point p_cam  
            except Exception as e:
                self.get_logger().warn(f"Unable to convert from {src_frame} to {global_frame}: {e}")
                return
            
            # Search for objects of the same class near the new observation
            if classe not in self.objects:
                self.objects[classe] = {} 

            matched_id = None

            # If it detects two identical objects less than 1m apart, it records them only once. 
            for obj_id, obj in self.objects[classe].items():
                pos = obj['avg'] 
                diffx = p_map.point.x - pos[0]
                diffy = p_map.point.y - pos[1]
                diffz = p_map.point.z - pos[2]

                dist = math.sqrt(diffx**2 + diffy**2 + diffz**2)
                if dist < 1.0: # distance between two detected objects to determine whether they are the same object
                    matched_id = obj_id
                    break        

            if matched_id is None:
                # New object
                self.id += 1
                new_id = str(f"id{self.id}")
                self.objects[classe][new_id] = {
                    'positions': (p_map.point.x, p_map.point.y, p_map.point.z),
                    'avg': (p_map.point.x, p_map.point.y, p_map.point.z),
                    'n': 1,
                    'last_seen': self.get_clock().now()
                }
                matched_id = new_id
            else:
                # Existing object
                self.objects[classe][matched_id]['n'] += 1
                self.objects[classe][matched_id]['avg'] = self.compute_average((p_map.point.x, p_map.point.y, p_map.point.z), self.objects[classe][matched_id]['positions'])
                self.objects[classe][matched_id]['positions'] = (p_map.point.x, p_map.point.y, p_map.point.z)
                self.objects[classe][matched_id]['last_seen'] = self.get_clock().now()

        except Exception as e:
            self.get_logger().error(f"Errore in box_callback: {e}")
    
    def publish_markers(self):  
        for classe, objects in self.objects.items():
            for obj_id, obj in objects.items():
                n = obj['n']
                if n > 100: # minimum number of observations for each object before inserting the marker (30)
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f"class_{classe}"
                    marker.id = int(obj_id.replace('id', ''))
                    if classe == 0:   # Cone (Cylinder because there are no cones in Rviz)
                        marker.type = Marker.CYLINDER
                    elif classe == 1: # Cube
                        marker.type = Marker.CUBE
                    elif classe == 2: # Sphere
                        marker.type = Marker.SPHERE 
                    marker.action = Marker.ADD 

                    # Marker position
                    avg = obj['avg']
                    marker.pose.position.x = avg[0]
                    marker.pose.position.y = avg[1]
                    marker.pose.position.z = avg[2]

                    # Scale and color
                    marker.scale.x = 0.5
                    marker.scale.y = 0.5
                    marker.scale.z = 0.5

                    marker.color.r = 1.0
                    marker.color.g = 0.1
                    marker.color.b = 0.0
                    marker.color.a = 0.8

                    self.marker_pub.publish(marker)
        
        # Return to the desired final position
        if self.robots_sent_home==False:  # Check if you haven't already done so.
            if self.check_all_objects_found():
                self.sim_time = (self.get_clock().now()-self.start_time).nanoseconds * 1e-9 
                self.get_logger().warn(f"Start Time: {self.start_time} End Time {self.get_clock().now()})")
                # Exploration time
                self.get_logger().warn(f"All items found! Return of the robots. Time: {self.sim_time} (ns {self.get_clock().now()-self.start_time})") 
                self.send_robots_home()
                self.robots_sent_home = True 

def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == '__main__':
    main()