import rclpy
from rclpy.node import Node

from ultralytics import YOLO
from ultralytics.engine.results import Boxes
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from custom_msg.msg import Box

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # Read the robot namespace
        self.declare_parameter('namespace', 'robot')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # Load YOLO models
        self.get_logger().info("Loading YOLO models...")
        self.detection_model = YOLO("runs/detect/train/weights/best.pt")

        # CvBridge for converting between ROS Image and OpenCV (numpy)
        self.bridge = CvBridge()

        # Initialization of variables
        self.depth_image = None
        self.distance = 0.0

        # Subscriber
        camera_topic =f'/{self.namespace}/camera/image_raw'
        self.get_logger().info(f"Topic: {camera_topic}")
        self.create_subscription(
            Image,
            camera_topic, 
            self.image_callback,
            5
        )
        self.get_logger().info('Ultralytics node initialized. Waiting for images...')

        # Subscriber to depth image
        camera_depth_topic =f'/{self.namespace}/camera/depth/image_raw'
        self.create_subscription(
            Image,
            camera_depth_topic,
            self.depth_callback,
            5
        )

        # Publisher of annotated images
        topic_detection = f'/{self.namespace}/ultralytics/detection/image'
        self.det_image_pub = self.create_publisher(Image, topic_detection, 5)

        # Publisher of information obtained with yolo (class, xc, yc, w, h, distance)
        self.det_pub = self.create_publisher(Box, f'/{self.namespace}/yolo/detections', 10)

    def depth_callback(self, msg):
        # Save the last depth image received 
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            return

    def image_callback(self, msg):
        # Callback called for each image received
        frame_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Detection (only if there are subscribers to save resources)
        if self.det_image_pub.get_subscription_count() > 0:
            det_result = self.detection_model(frame_rgb) # apply yolo to the image

            boxes = det_result[0].boxes  # Boxes object containing the detected bounding boxes
            if boxes is not None:
                xs = boxes.xyxy.cpu().numpy()  # if on GPU, switch to CPU and numpy
                confs = boxes.conf.cpu().numpy()
                classes = boxes.cls.cpu().numpy()

                for i, (xy, conf, cls) in enumerate(zip(xs, confs, classes)):
                    # display of all detected boxes (even if invalid)
                    self.get_logger().info(f"Det {i}: classe={int(cls)}, conf={float(conf):.2f}, xyxy={xy.tolist()}")
                
                # xyxy = x1 y1 x2 y2
                # x1: X coordinate of the top left corner of the box - pixel column where the box begins
                # y1: Y coordinate of the top left corner of the box - pixel row where the box begins
                # x2: X coordinate of the bottom right corner of the box - pixel column where the box ends
                # y2: Y coordinate of the bottom right corner of the box - pixel row where the box ends

                # Select only boxes belonging to classes 0, 1, and 2 and with confidence greater than 0.90.
                valid_idx = [
                    i for i, (cls, conf) in enumerate(zip(classes, confs))
                    if int(cls) in [0, 1, 2] and conf > 0.90 
                ]

                # If there are valid boxes
                if valid_idx:
                    filtered_boxes = Boxes(
                        boxes=boxes.data[valid_idx],
                        orig_shape=det_result[0].orig_shape
                    )

                    # Temporarily replace in the result
                    det_result[0].boxes = filtered_boxes

                    # Filter boxes 
                    det_annotated = det_result[0].plot(show=False)
                    ros_img = self.bridge.cv2_to_imgmsg(det_annotated, encoding='rgb8')
                    ros_img.header = msg.header
                    
                    self.det_image_pub.publish(ros_img)
    
            for box in det_result[0].boxes:
                xywh = box.xywh[0].tolist()
                xc = int(xywh[0])
                yc = int(xywh[1])

                # Verify that the depth image is available
                if self.depth_image is None:
                    self.get_logger().warn("No depth image available yet.")
                    return
                
                try:
                    # Avoid index errors outside the image boundaries
                    height, width = self.depth_image.shape[:] # image depth dimensions
                    if xc < 0 or yc < 0 or xc >= width or yc >= height:
                        self.get_logger().warn(f"Coordinates outside the image: ({xc},{yc}) not in (0-{width},{height})")
                        continue
                    else:
                        self.get_logger().info(f"Correct coordinates (center coordinates: {xc},{yc})") 

                    # Extract the distance in meters from the corresponding pixel
                    self.distance = float(self.depth_image[yc, xc])

                except Exception as e:
                    self.get_logger().error(f"Distance calculation error: {e}")

                # Message posted in the topic
                det_msg = Box()
                det_msg.header.frame_id = f"{self.namespace}/camera_depth_optical_frame"
                det_msg.header.stamp = self.get_clock().now().to_msg()
                det_msg.classe = int(box.cls)
                det_msg.xc = xywh[0]
                det_msg.yc = xywh[1]
                det_msg.w = xywh[2]
                det_msg.h = xywh[3]
                det_msg.distance = self.distance
                self.det_pub.publish(det_msg)                

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()

if __name__ == "__main__":
    main()