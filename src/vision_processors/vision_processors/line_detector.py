#!/usr/bin/env python

###########
# IMPORTS #
###########
import numpy as np
import rclpy
from rclpy.node import Node
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_interfaces.msg import Line
import sys

#############
# CONSTANTS #
#############
LOW = 225  # Lower image thresholding bound
HI = 255   # Upper image thresholding bound
LENGTH_THRESH = 200  # If the length of the largest contour is less than LENGTH_THRESH, we will not consider it a line
KERNEL = np.ones((5, 5), np.uint8)
DISPLAY = True

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')

        # A subscriber to the topic '/aero_downward_camera/image'
        self.camera_sub = self.create_subscription(
            Image,
            '/down_camera/image_raw',
            self.camera_sub_cb,
            10
        )
        # A publisher which will publish a parametrization of the detected line to the topic '/line/param'
        self.line_pub = self.create_publisher(Line, 'line', 1)

        # A publisher which will publish an image annotated with the detected line to the topic 'line/detector_image'
        self.processed_pub = self.create_publisher(Image, 'image_processed', 1)

        # Initialize instance of CvBridge to convert images between OpenCV images and ROS images
        self.bridge = CvBridge()

    ######################
    # CALLBACK FUNCTIONS #
    ######################
    def camera_sub_cb(self, msg):
        """
        Callback function which is called when a new message of type Image is received by self.camera_sub.
            Args: 
                - msg = ROS Image message
        """
        try:
            # Convert Image msg to OpenCV image
            image = self.bridge.imgmsg_to_cv2(msg, "mono8")

            # Detect line in the image. detect returns a parameterize the line (if one exists)
            line = self.detect_line(image)

            # If a line was detected, publish the parameterization to the topic '/line/param'
            if line is not None:
                msg = Line()
                msg.x, msg.y, msg.vx, msg.vy = line
                # Publish param msg
                self.line_pub.publish(msg)

            # Publish annotated image if DISPLAY is True and a line was detected
            if DISPLAY and line is not None:
                # Draw the detected line on a color version of the image
                annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                x, y, vx, vy = line
                pt1 = (int(x - 100*vx), int(y - 100*vy))
                pt2 = (int(x + 100*vx), int(y + 100*vy))
                cv2.line(annotated, pt1, pt2, (0, 0, 255), 2)
                cv2.circle(annotated, (int(x), int(y)), 5, (0, 255, 0), -1)
                # Convert to ROS Image message and publish
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                self.processed_pub.publish(annotated_msg)
        
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        except Exception as e:
            self.get_logger().error(f"Error in camera callback: {e}")

    ##########
    # DETECT #
    ##########
    def detect_line(self, image):
        """ 
        Given an image, fit a line to biggest contour if it meets size requirements (otherwise return None)
        and return a parameterization of the line as a center point on the line and a vector
        pointing in the direction of the line.
            Args:
                - image = OpenCV image
            Returns: (x, y, vx, vy) where (x, y) is the centerpoint of the line in image and 
            (vx, vy) is a vector pointing in the direction of the line. Both values are given
            in downward camera pixel coordinates. Returns None if no line is found
        """
        try:
            # Step 1: Threshold to isolate dark line on light background
            _, binary = cv2.threshold(image, LOW, 255, cv2.THRESH_BINARY_INV)        # Step 2: Apply morphological operations to remove noise and fill gaps
            binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, KERNEL)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, KERNEL)        # Step 3: Find external contours
            contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                self.direction_initialized = False
                return None        # Step 4: Find largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)        # Step 5: Check minimum contour arc length
            contour_length = cv2.arcLength(largest_contour, closed=False)
            if contour_length < LENGTH_THRESH:
                self.direction_initialized = False
                return None        # Step 6: Fit line to the largest contour
            line_params = cv2.fitLine(largest_contour, cv2.DIST_L2, 0, 0.01, 0.01)
            vx_raw = float(line_params[0])
            vy_raw = float(line_params[1])
            x = float(line_params[2])
            y = float(line_params[3])        # Step 7: Ensure direction consistency (flip if necessary)
            if self.direction_initialized:
                dot_raw = self.prev_vx * vx_raw + self.prev_vy * vy_raw
                dot_flipped = self.prev_vx * -vx_raw + self.prev_vy * -vy_raw
                if dot_raw >= dot_flipped:
                    vx, vy = vx_raw, vy_raw
                else:
                    vx, vy = -vx_raw, -vy_raw
            else:
                if vy_raw < 0:
                    vx, vy = -vx_raw, -vy_raw
                else:
                    vx, vy = vx_raw, vy_raw
                self.direction_initialized = True        # Step 8: Save current direction for consistency
            self.prev_vx = vx
            self.prev_vy = vy
            return (x, y, vx, vy)
        except Exception as e:
            self.get_logger().error(f"Error in detect_line: {e}")
            self.direction_initialized = False
            return None

def main(args=None):
    rclpy.init(args=args)
    detector = LineDetector()
    detector.get_logger().info("Line detector initialized")
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as e:
        print(e)
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()