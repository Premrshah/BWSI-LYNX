import cv2
from cv2 import aruco
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image


class ArDetector(Node):
    def __init__(self):
        self.camera_sub = self.create_subscription(
            Image,
            '/front_cam/image_raw',
            self.camera_sub_cb,
            10
        )
        
        self.processed_pub = self.create_publisher(
            Image,
            'image_processed',
            10
        )
        
        self.ar_pub = None
        
        self.bridge = CvBridge()
        
        self.detector = aruco.ArucoDetector(aruco.getPredefinedDictionary(aruco.DICT_5X5_100))

    
    def camera_sub_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg)
        
        (corners__fc, ids, _) = self.detector.detectMarkers(image)
        
        processed_img = aruco.drawDetectedCornersCharuco(image, corners__fc[0], ids, (0, 0, 255))
        self.processed_pub.publish(self.bridge.cv2_to_imgmsg(processed_img))
        
        
    
def main(args=None):
    rclpy.init(args=args)
    ar_detector = ArDetector()
    ar_detector.get_logger().info("AR detector initialized")
    try:
        rclpy.spin(ar_detector)
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(e)
    finally:
        ar_detector.destroy_node()
        rclpy.shutdown()
