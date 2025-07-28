import cv2
from cv2 import aruco
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from vision_interfaces.msg import CamToTagTf
import numpy as np

CAMERA_MATRIX = np.array(
    [[1251.909096, 0.000000,    380.379727],
     [0.000000,    1235.038409, 290.901704],
     [0.000000,    0.000000,    1.000000  ]],
    dtype=np.float32
)

DISTORTION = np.array([0.219093, -0.586732, -0.004906, -0.055902, 0.000000,])

MARKER_SIZE = 0.2667    # meters (converted from 10.5")

ARUCO_POINTS = np.array(
    [[-MARKER_SIZE / 2, MARKER_SIZE / 2, 0],#corners of AR in tag frame
     [MARKER_SIZE / 2, MARKER_SIZE / 2, 0],
     [MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
     [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]],
    dtype=np.float32
)







class ArDetector(Node):
    def __init__(self, cam_matrix = None):
        super().__init__('ar_detector')

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
        
        self.transform_pub = self.create_publisher(
            CamToTagTf,
            'cam_to_tag',
            10
        )

        self.bridge = CvBridge()

        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.detector = aruco.ArucoDetector(aruco_dict)

    def camera_sub_cb(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = self.detector.detectMarkers(image)

        if ids is not None:
            image = aruco.drawDetectedMarkers(image, corners, ids)

        self.processed_pub.publish(self.bridge.cv2_to_imgmsg(image, encoding='bgr8'))
        
        for (tag, id) in (corners, ids):
            (_, R_tag2cam, p_tag_cam__cam) = cv2.solvePnP(ARUCO_POINTS, tag, CAMERA_MATRIX, DISTORTION, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            R_cam2tag = np.linalg.inv(R_tag2cam)
            p_cam_tag__tag = R_cam2tag @ -p_tag_cam__cam 
            transform = CamToTagTf()
            transform.tag_id = id
            transform.r11 = R_cam2tag[0][0]
            transform.r12 = R_cam2tag[0][1]
            transform.r13 = R_cam2tag[0][2]
            transform.r21 = R_cam2tag[1][0]
            transform.r22 = R_cam2tag[1][1]
            transform.r23 = R_cam2tag[1][2]
            transform.r31 = R_cam2tag[2][0]
            transform.r32 = R_cam2tag[2][1]
            transform.r33 = R_cam2tag[2][2]
            transform.t1 = p_cam_tag__tag[0]
            transform.t2 = p_cam_tag__tag[1]
            transform.t3 = p_cam_tag__tag[2]
            self.transform_pub.publish(transform)

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
