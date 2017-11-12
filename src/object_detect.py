#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

# Desired shape colors for object detector (HSV)
COLOR_BOUNDS = [
   ([30, 100, 70], [90, 255, 255]), # GREEN
   ([100, 100, 70], [130, 255, 255]), # BLUE
   ([0, 100, 70], [20, 255, 255]) # ORANGE
]

class ImageStreamConverter:
    """
    Open CV and ROS image stream converter. Responsible for subscribing to
    camera topics and publishing new topic. The primary function is allowing
    intermediate OpenCV image manipulation, through the cv_image_listener
    initializer parameter. Publishes post-processed image on
    /sticky_wickets/object/image.
    """
    def __init__(self,
                 cv_image_listener,
                 pub_topic="/sticky_wickets/object/image", # NOTE: This is a topic is for testing
                 sub_topic="camera/rgb/image_raw"):
        self.image_sub = rospy.Subscriber(sub_topic, Image, self.img_callback)
        self.image_pub = rospy.Publisher(pub_topic, Image, queue_size=10)
        self.image_listener = cv_image_listener
        self.bridge = CvBridge()

    def img_callback(self, ros_img):
        """
        ROS camera stream image callback. Converts ROS image to CV image,
        calls the listener, and republishes on new ROS topic.
        """
        cv_img = self.convert_rosimage(ros_img)

        if cv_img is not None:
            cv_img = self.image_listener(cv_img)
            ros_img = self.convert_cvimage(cv_img)

            if ros_img is not None:
                self.image_pub.publish(ros_img)

    def convert_cvimage(self, cv_image):
        """Converts OpenCV image to ROS"""
        try:
            return self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            print("ERROR: Could not convert open_cv image. ", e)

    def convert_rosimage(self, ros_image):
        """Converts ROS image to OpenCV"""
        try:
            return self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            print("ERROR: Could not convert ros image. ", e)


class ObjectDetector:
    """
    Basic object detector. Detects objects using basic shape and color
    attributes. This is fully decoupled from ROS's image stream,
    and solely relies on OpenCV for image analysis and detection.
    It obtains the OpenCV Image stream via an ImageStreamConverter
    instance which converts ROS to OpenCV. Detected objects bounding
    boxes are published on /sticky_wickets/object_data
    """
    def __init__(self, colors, min_sz=14, max_sz=300):
        self.cv_converter = ImageStreamConverter(self.detect_objects)
        self.object_pub = rospy.Publisher("/sticky_wickets/object_data", String, queue_size=10)
        self.color_bounds = colors
        self.min_sz = min_sz
        self.max_sz = max_sz

    def detect_objects(self, cv_image):
        """
        ImageStreamConverter listener callback.
        Processes CV image, looking for various objects.
        """
        color_masked_img = self.isolate_objects_by_color(cv_image)

        squares = self.process_polygons(color_masked_img, 4)
        triangl = self.process_polygons(color_masked_img, 3)
        circles = self.process_circles(color_masked_img)

        # Publish detected bounding rectangles (string message type)
        pub_shapes = str({ "4": squares, "0": circles, "3": triangl })
        self.object_pub.publish(pub_shapes)
        self.visualize_objects(color_masked_img, squares + triangl + circles)

        return color_masked_img

    def visualize_objects(self, cv_image, rects):
        """
        Visualizes given objects on the given CV Image
        """
        for (x, y, w, h) in rects:
            cv2.rectangle(cv_image, (x,y),(x+w,y+h), (200,100,200), 8)

    def process_polygons(self, cv_image, polygon):
        """
        Detects polygonal objects in an image. To reduce noise, the
        image given should be pre-processed to isolate / highlight
        shapes of interest (this is done using color processing).
        """
        cv_img_grey = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        ret, thresh = cv2.threshold(cv_img_grey, 127, 255, 1)
        _, contours, h = cv2.findContours(thresh, 1, 2)
        # Be careful with this constant: it is an obscure parameter
        # of approxPolyDP that determines how irregular detected shapes
        # should be. Figured out good value heuristically.
        epsilon_approx = 0.031

        detected_objs = []

        for cnt in contours:
            approx_poly = cv2.approxPolyDP(cnt,epsilon_approx*cv2.arcLength(cnt,True),True)
            (x, y, w, h) = cv2.boundingRect(approx_poly)

            if len(approx_poly) == polygon:
                if w > self.min_sz and h > self.min_sz and w < self.max_sz and h < self.max_sz:
                    detected_objs.append((x, y, w, h))

        return detected_objs

    def process_circles(self, cv_image):
        """
        Detects circular objects in an image. To reduce noise, the
        image given should be pre-processed to isolate / highlight
        shapes of interest (this is done using color processing).
        """
        cv_img_grey = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)

        ret, thresh = cv2.threshold(cv_img_grey, 127, 255, 1)
        _, contours, h = cv2.findContours(thresh, 1, 2)
        min_cirularity = 0.79
        max_circularity = 1.2

        detected_circles = []

        for cnt in contours:
            perimeter = cv2.arcLength(cnt, True)
            area = cv2.contourArea(cnt)
            if perimeter == 0:
                continue

            circularity = 4*math.pi*(area/(perimeter*perimeter))
            if min_cirularity < circularity < max_circularity:
                (x, y, w, h) = cv2.boundingRect(cnt)
                if w > self.min_sz and h > self.min_sz and w < self.max_sz and h < self.max_sz:
                    detected_circles.append((x, y, w, h))

        return detected_circles

    def isolate_objects_by_color(self, cv_image, rects=0):
        """
        Analyzes colors within the given cv_image. If desired colors
        are located in the image, this would indicate the presence of
        objects of interest. Multiple masks are applied to the image
        to isolate these objects. The processed image can then be handled
        by polygon detector methods for contour analysis.
        """
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        total_mask = 0

        for (lower_bound, upper_bound) in self.color_bounds:
            lower_bound = np.array(lower_bound, dtype="uint8")
            upper_bound = np.array(upper_bound, dtype="uint8")

            # Mask the images in the current color bounds, then combine with
            # the previous mask of previous bounds
            mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
            total_mask = total_mask | mask

        image = cv2.bitwise_and(hsv_image, hsv_image, mask=total_mask)

        return image;


def main(args):

    detector = ObjectDetector(COLOR_BOUNDS)
    rospy.init_node('object_detect', anonymous=True)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
