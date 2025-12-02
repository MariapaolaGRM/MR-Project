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
        
        # Legge il namespace del robot
        self.declare_parameter('namespace', 'robot')
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value

        # Carica i modelli YOLO
        self.get_logger().info("Caricamento modelli YOLO...")
        self.detection_model = YOLO("runs/detect/train/weights/best.pt")

        # CvBridge per convertire tra ROS Image e OpenCV (numpy)
        self.bridge = CvBridge()

        # Inizializzazione variabili
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
        self.get_logger().info('Ultralytics node inizializzato. In attesa di immagini...')

        # Subscriber alla depth image
        camera_depth_topic =f'/{self.namespace}/camera/depth/image_raw'
        self.create_subscription(
            Image,
            camera_depth_topic,
            self.depth_callback,
            5
        )

        # Publisher delle immagini annotate
        topic_detection = f'/{self.namespace}/ultralytics/detection/image'
        self.det_image_pub = self.create_publisher(Image, topic_detection, 5)

        # Publisher delle informazioni ricavate con yolo (classe, xc, yc, w, h, distance)
        self.det_pub = self.create_publisher(Box, f'/{self.namespace}/yolo/detections', 10)

    def depth_callback(self, msg):
        # Salva l'ultima immagine di profondità ricevuta 
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = depth
        except Exception as e:
            self.get_logger().error(f"Errore conversione depth: {e}")
            return

    def image_callback(self, msg):
        # Callback chiamato ad ogni immagine ricevuta.
        frame_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Detection (solo se ci sono subscriber per risparmiare risorse)
        if self.det_image_pub.get_subscription_count() > 0:
            det_result = self.detection_model(frame_rgb) 

            boxes = det_result[0].boxes  # oggetto Boxes che contiene le bounding box rilevate
            if boxes is not None:
                xs = boxes.xyxy.cpu().numpy()  # se su GPU, passare a CPU e numpy
                confs = boxes.conf.cpu().numpy()
                classes = boxes.cls.cpu().numpy()

                for i, (xy, conf, cls) in enumerate(zip(xs, confs, classes)):
                    self.get_logger().info(f"Det {i}: classe={int(cls)}, conf={float(conf):.2f}, xyxy={xy.tolist()}")
                
                # xyxy = x1 y1 x2 y2
                # x1 coordinata X del punto in alto a sinistra del box - colonna pixel da cui inizia la box
                # y1 coordinata Y del punto in alto a sinistra del box - riga pixel da cui inizia la box
                # x2 coordinata X del punto in basso a destra del box - colonna pixel dove finisce la box
                # y2 coordinata Y del punto in basso a destra del box - riga pixel dove finisce la box

                valid_idx = [
                    i for i, (cls, conf) in enumerate(zip(classes, confs))
                    if int(cls) in [0, 1, 2] and conf > 0.90 #0.85
                ]

                # Se ci sono box valide
                if valid_idx:
                    filtered_boxes = Boxes(
                        boxes=boxes.data[valid_idx],
                        orig_shape=det_result[0].orig_shape
                    )

                    # Sostituisci temporaneamente nel risultato
                    det_result[0].boxes = filtered_boxes

                    # Box filtrate 
                    det_annotated = det_result[0].plot(show=False)
                    ros_img = self.bridge.cv2_to_imgmsg(det_annotated, encoding='rgb8')
                    ros_img.header = msg.header
                    
                    self.det_image_pub.publish(ros_img)

                
            for box in det_result[0].boxes:
                xywh = box.xywh[0].tolist()
                xc = int(xywh[0])
                yc = int(xywh[1])

                # Verifica che l'immagine di profondità sia disponibile
                if self.depth_image is None:
                    self.get_logger().warn("Nessuna depth image disponibile ancora.")
                    return
                
                try:
                    # Evita errori di indice fuori dai limiti dell'immagine
                    height, width = self.depth_image.shape[:]
                    if xc < 0 or yc < 0 or xc >= width or yc >= height:
                        self.get_logger().warn(f"Coordinate fuori immagine: ({xc},{yc}) non in (0-{width},{height})")
                        continue
                    else:
                        self.get_logger().info(f"Coordinate corrette, ({xc},{yc})") 

                    # Estrai la distanza in metri dal pixel corrispondente
                    self.distance = float(self.depth_image[yc, xc])
    

                except Exception as e:
                    self.get_logger().error(f"Errore calcolo distanza: {e}")

                # Messaggio pubblicato nel topic
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
    rclpy.shutdown()


if __name__ == "__main__":
    main()