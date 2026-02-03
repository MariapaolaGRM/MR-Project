import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import math
import time
import tf2_geometry_msgs

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose

class GoalNode(Node):
    def __init__(self):
        super().__init__('goal_node')

        # Read the robot namespace
        self.declare_parameter('namespace', 'robot')
        self.ns= self.get_parameter('namespace').get_parameter_value().string_value
        
        # State variables 
        self.last_move_time = time.time()
        self.last_pose = None
        self.goal_sent = False

        # Node status (active or deactivated)
        self.is_active = True
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            f"/{self.ns}/odom",
            self.odom_callback,
            10
        )

        # Subscriber to check node activation
        self.control_sub = self.create_subscription(
            Bool,
            f"/{self.ns}/goal_node/active",
            self.control_callback,
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

        # Timer to check stall 
        timer_period = 1  # Timer at 1s
        self.timer = self.create_timer(timer_period, self.check)

    # Callback to activate/deactivate the node
    def control_callback(self, msg: Bool):
        self.is_active = msg.data
        if self.is_active==False:
            self.get_logger().warn(f"[{self.ns}] GoalNode DISABLED - deletion of active goals")
            # Reset status
            self.goal_sent = False

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.sqrt(vx*vx + vy*vy)

        # Save last pose
        self.last_pose = msg.pose.pose

        # If it has moved -> update timestamp
        if v > 0.05:
            self.last_move_time = time.time()
    
    def check(self):
        # Check if the node is active before doing anything.
        if self.is_active == False:
            return  # It does not matter if it is disabled
        
        stalled = False
        self.get_logger().warn(f"Stalled {stalled}")

        if self.last_pose is None: # If the last position is empty -> do nothing
            self.get_logger().warn(f"Last pose {self.last_pose}")
            return

        if (time.time() - self.last_move_time) > 50.0: # If the robot has been stationary for 50 seconds
            stalled = True

        if stalled:
            self.get_logger().warn(f"[{self.ns}] Robot appears stalled! Sending goal…")
            if self.goal_sent: # True
                # Send another goal 
                self.goal_sent = False
                self.send_goal(value = -1.0)
                self.resume_exploration
                return
            else:
                # Send the first goal
                self.goal_sent = True
                self.send_goal(value = 1.0)
                self.resume_exploration
                return

    def send_goal(self, value):
        pose_odom = PoseStamped()
        pose_odom.header.frame_id = f"{self.ns}/odom"
        pose_odom.header.stamp = self.get_clock().now().to_msg()
        pose_odom.pose = self.last_pose  # position in odom (self.last_pose is a Pose)

        # Turn the pose into a frame map
        try:
            pose_map = self.tf_buffer.transform(
                pose_odom,
                f"{self.ns}/map",
                timeout=rclpy.duration.Duration(seconds=1)
            )

        except Exception as e:
            self.get_logger().error(f"TF transform failed: {e}")
            return

        # Only change the Y in the frame map
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

if __name__ == '__main__':
    main()