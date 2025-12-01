import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import math
import time

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')

        # Legge il namespace del robot
        self.declare_parameter('namespace', 'robot')
        self.ns= self.get_parameter('namespace').get_parameter_value().string_value
        
        # State variables 
        self.last_move_time = time.time()
        self.last_pose = None
        self.goal_sent = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            f"/{self.ns}/odom",
            self.odom_callback,
            10
        )

        # Explorer resume publisher 
        self.resume_pub = self.create_publisher(
            Bool,
            f"/{self.ns}/explore/resume",
            10
        )

        # Nav2 Action Client 
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            f"/{self.ns}/navigate_to_pose"
        )

        # Timer per controllare stallo 
        timer_period = 1  # Timer a 1s
        self.timer = self.create_timer(timer_period, self.check)

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.sqrt(vx*vx + vy*vy)

        # Salva ultima posa
        self.last_pose = msg.pose.pose

        # Se si è mosso -> aggiorna timestamp
        if v > 0.05:
            self.last_move_time = time.time()
    
    def check(self):
        stalled = False
        self.get_logger().warn(f"Stalled {stalled}")

        if self.last_pose is None: # Se l'ultima posizione è vuota -> fare nulla
            self.get_logger().warn(f"Last pose {self.last_pose}")
            return

        if (time.time() - self.last_move_time) > 40.0: # Se robot sta fermo da 40 secondi
            stalled = True

        if stalled:
            self.get_logger().warn(f"[{self.ns}] Robot appears stalled! Sending goal…")
            if self.goal_sent: # True
                # Invia un altro goal 
                self.goal_sent = False
                self.resume_exploration()
                self.send_goal(value = -0.5)
                return
            else:
                # Invia il primo goal
                self.goal_sent = True
                self.resume_exploration()
                self.send_goal(value = 0.5)
                return

    def send_goal(self, value):
        # Costruisci un PoseStamped 
        pose_odom = PoseStamped()
        pose_odom.header.frame_id = f"{self.ns}/odom"
        pose_odom.header.stamp = self.get_clock().now().to_msg()
        pose_odom.pose = self.last_pose  # posizione in odom (self.last_pose è un Pose)

        # Trasforma la posa in frame map
        try:
            pose_map = self.tf_buffer.transform(
                pose_odom,
                f"{self.ns}/map",
                timeout=rclpy.duration.Duration(seconds=1)
            )

            #transform = self.tf_buffer.lookup_transform(
            #    f"{self.ns}/map", # to
            #    f"{self.ns}/odom", # from
            #    rclpy.time.Time()
            #)
            #self.get_logger().info(f"{pose_odom.pose}")
            #pose_map = do_transform_pose(pose_odom, transform)
        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return

        # Modifica solo la Y nel frame map
        pose_map.pose.position.y += value

        goal = NavigateToPose.Goal()
        goal.pose = pose_map

        self.nav_client.wait_for_server()
        self.nav_client.send_goal_async(goal)

        self.get_logger().info(f"[{self.ns}] Sent goal in map frame")
    
    def resume_exploration(self):
        msg = Bool()
        msg.data = True
        self.resume_pub.publish(msg)
        self.get_logger().info(f"[{self.ns}] Sent explore/resume TRUE")

def main(args=None):
    rclpy.init(args=args)
    node = GoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()