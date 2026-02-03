#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from nav_msgs.msg import OccupancyGrid

# Publish static transformation map -> <robot>/map 
class MapStaticBroadcaster(Node):
    def __init__(self):
        super().__init__('map_static_broadcaster')

        # List of robots
        self.declare_parameter('robots', ['robot1','robot2'])
        self.robot_names = self.get_parameter('robots').get_parameter_value().string_array_value

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Subscriber on /map for each robot
        self.robot_subscriptions = []
        for robot in self.robot_names:
            topic = f'/{robot}/map'
            self.create_subscription(
                OccupancyGrid, topic, lambda msg, r=robot: self.map_callback(msg, r), 10
            )
            self.get_logger().info(f"Subscribed to {topic}")
      
    def map_callback(self, msg: OccupancyGrid, robot: str):
        self.get_logger().info(f"x:{msg.info.origin.position.x}, y:{msg.info.origin.position.y}")

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'{robot}/map'
        t.transform.translation.x = msg.info.origin.position.x
        t.transform.translation.y = msg.info.origin.position.y
        t.transform.translation.z = msg.info.origin.position.z
        t.transform.rotation = msg.info.origin.orientation

        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published static transform: map -> {robot}/map")

def main(args=None):
    rclpy.init(args=args)
    node = MapStaticBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()