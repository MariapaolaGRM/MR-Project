import rclpy
from rclpy.node import Node

import math
from custom_msg.msg import Box
from typing import Dict, Tuple

import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped, PoseStamped
from visualization_msgs.msg import Marker

from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class CentralNode(Node):
    def __init__(self):
        super().__init__('central_node')
        
        self.declare_parameter('robots', ['robot1','robot2'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value 

        # Subscriber su /yolo/detections per ogni robot
        for robot in self.robot_names:
            topic = f'/{robot}/yolo/detections'
            self.create_subscription(
                Box, topic, lambda msg, r=robot: self.box_callback(msg, r), 10
            )
            self.get_logger().info(f"Subscribed to {topic}") 

        # struttura dati (dizionario): objects[classe][id] -> {"avg": (x,y,z), "positions": [(x,y,z),...], "n": int}
        self.objects: Dict[int, Dict[str, Dict]] = {}
        self.id = 0

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Posizioni iniziali robot
        #self.declare_parameter('positions', [[2.5, -3.0], [2.5, 3.0]])
        #self.robot_positions = self.get_parameter('positions').get_parameter_value().string_array_value 
        self.robot_positions = [[2.5, -3.0], [2.5, 3.0]]
        
        # Publisher Marker Rviz  
        self.marker_pub = self.create_publisher(Marker, '/detected_objects_markers', 10)
        self.create_timer(0.5, self.publish_markers)  # aggiorna ogni 0.5s

        # Action per far tornare i robot in una posizione scelta 
        # Funzionante solo con house2_model
        self.nav_clients = {}   # dizionario robot → ActionClient

        for robot in self.robot_names:
            topic = f'/{robot}/navigate_to_pose'
            self.nav_clients[robot] = ActionClient(
                self,
                NavigateToPose,
                topic
            )
            self.get_logger().info(f"Created ActionClient for {robot} on {topic}")

    def pixel_depth_to_camera_point(self, u: float, v: float, depth_m: float) -> Tuple[float,float,float]:
        # Converti (u,v,depth) in coordinate camera usando modello pinhole

        # Parametri della camera: fx, fy, cx, cy (ricavati dal topic camera_info)
        fx = 320.25492609007654
        fy = 320.25492609007654
        cx = 320.0
        cy = 240.0        
        
        # Calcolo posizione 3D dell'oggetto nel sistema di riferimento della camera
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        z = depth_m
        return (x, y, z)
    
    def compute_average(self, pmap: Tuple[float,float,float], positions: Tuple[float,float,float]) -> Tuple[float,float,float]:
        alpha = 0.6 # confidenza per la media pesata
        
        sx = positions[0]*alpha + pmap[0]*(1-alpha)
        sy = positions[1]*alpha + pmap[1]*(1-alpha)
        sz = positions[2]*alpha + pmap[2]*(1-alpha)
        return (sx, sy, sz)
    
    def send_nav_goal(self, robot, x, y): 
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
        
    def box_callback(self, msg: Box, robot: str):
        try:
            # Lettura valori dal messaggio
            src_frame = msg.header.frame_id
            classe = int(msg.classe)
            xc = float(msg.xc) 
            yc = float(msg.yc) 
            depth_m = float(msg.distance) # Distanza dall'oggetto nel frame del robot

            # Verifica depth valida 
            if depth_m <= 0.0 or math.isinf(depth_m) or math.isnan(depth_m):
                self.get_logger().warn(f"[{robot}] depth non valida: {depth_m}")
                return

            # Calcolo posizione 3D dell'oggetto nel sistema di riferimento della camera
            cam_x, cam_y, cam_z = self.pixel_depth_to_camera_point(xc, yc, depth_m)

            # Listener della trasformazione tra coordinate oggetto nel frame optical e coordinate nel frame map
            p_cam = PointStamped() 
            p_cam.header.frame_id = src_frame
            p_cam.point.x = cam_x
            p_cam.point.y = cam_y
            p_cam.point.z = cam_z

            try:
                global_frame = 'map'

                trans = self.tf_buffer.lookup_transform(global_frame, src_frame,rclpy.time.Time())
                p_map = do_transform_point(p_cam, trans) # applica la trasformazione trans al punto p_cam
                
            except Exception as e:
                self.get_logger().warn(f"Impossibile trasformare da {src_frame} a {global_frame}: {e}")
                return
            
            # Cerca oggetto della stessa classe vicino alla nuova osservazione
            if classe not in self.objects:
                self.objects[classe] = {} 

            matched_id = None

            # Se rileva 2 oggetti uguali a meno di 0.5m li registra una sola volta 
            for obj_id, obj in self.objects[classe].items():
                pos = obj['avg'] 
                diffx = p_map.point.x - pos[0]
                diffy = p_map.point.y - pos[1]
                diffz = p_map.point.z - pos[2]

                dist = math.sqrt(diffx**2 + diffy**2 + diffz**2)
                if dist < 0.5:
                    matched_id = obj_id
                    break        

            if matched_id is None:
                # Nuovo oggetto
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
                # Oggetto esistente
                self.get_logger().info(f"Oggetto esistente")
                self.objects[classe][matched_id]['n'] += 1
                self.objects[classe][matched_id]['avg'] = self.compute_average((p_map.point.x, p_map.point.y, p_map.point.z), self.objects[classe][matched_id]['positions'])
                self.objects[classe][matched_id]['positions'] = (p_map.point.x, p_map.point.y, p_map.point.z)
                self.objects[classe][matched_id]['last_seen'] = self.get_clock().now()

            
            # Torna in posizione scelta
            if len(self.objects) == 3 and len(self.objects[0]) == 2 and len(self.objects[1]) == 2 and len(self.objects[2]) == 2:
                count = 0
                for classe, obj in self.objects.items(): 
                    for obj_id, obj in self.objects[classe].items(): 
                        n = obj['n']
                        if n > 40: 
                            count+=1 # count conta quante rilevazioni soddisfano n>3
                            if count == 6:  
                                self.get_logger().warn("Ritorno dei robot alla posizione scelta...")
                                for i, robot in enumerate(self.robot_names):
                                    x, y = self.robot_positions[i]
                                    self.send_nav_goal(robot, x, y)

                                    #self.robot_positions = [[2.5, -3.0], [2.5, 3.0]]

        except Exception as e:
            self.get_logger().error(f"Errore in box_callback: {e}")
    
    def publish_markers(self):  
        for classe, objects in self.objects.items():
            for obj_id, obj in objects.items():
                n = obj['n']
                if n > 30:
                    marker = Marker()
                    marker.header.frame_id = 'map'
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = f"class_{classe}"
                    marker.id = int(obj_id.replace('id', ''))
                    if classe == 0:   # Cone (Cylinder perchè non esistono coni in Rviz)
                        marker.type = Marker.CYLINDER
                    elif classe == 1: # Cube
                        marker.type = Marker.CUBE
                    elif classe == 2: # Sphere
                        marker.type = Marker.SPHERE
                    marker.action = Marker.ADD 

                    # Posizione dei marker
                    avg = obj['avg']
                    marker.pose.position.x = avg[0]
                    marker.pose.position.y = avg[1]
                    marker.pose.position.z = avg[2]

                    # Scala e colore
                    marker.scale.x = 0.4
                    marker.scale.y = 0.4
                    marker.scale.z = 0.4

                    marker.color.r = 1.0
                    marker.color.g = 0.1
                    marker.color.b = 0.0
                    marker.color.a = 0.8

                    self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()