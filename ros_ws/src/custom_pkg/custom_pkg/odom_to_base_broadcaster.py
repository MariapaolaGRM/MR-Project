#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToBaseBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_to_base_broadcaster')

        # Lista dei robot da parametri
        self.declare_parameter('robots', ['robot1','robot2'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber su /<robot>/odom per ogni robot
        for robot in self.robot_names:
            topic = f'/{robot}/odom'
            self.create_subscription(
                Odometry, topic, lambda msg, r=robot: self.odom_callback(msg, r), 10
            )
            self.get_logger().info(f"Subscribed to {topic}")

    def odom_callback(self, msg: Odometry, robot: str):
        """Pubblica odom -> base_footprint dinamicamente"""
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = f"{robot}/odom"
        t.child_frame_id = f"{robot}/base_footprint"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f"Published {robot} odom -> base_footprint")


def main(args=None):
    rclpy.init(args=args)
    node = OdomToBaseBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()