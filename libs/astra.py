import rospy
import time
import cv2
from cv_bridge import CvBridge
from typing import *
from sensor_msgs.msg import Image


class RosAstraCamera:
    def __init__(self, rgb: str = "/camera/rgb/image_raw", dep: str = "/camera/depth/image_raw"):
        self.rgb_topic = rgb
        self.dep_topic = dep
        self.rgb = None
        self.dep = None
        if rgb or rgb is not None:
            rospy.Subscriber(self.rgb_topic, Image, self.callback_rgb)
            print("[astra.py] waiting for rgb")
            rospy.wait_for_message(self.rgb_topic, Image)

        if dep or dep is not None:
            rospy.Subscriber(self.dep_topic, Image, self.callback_dep)
            print("[astra.py] waiting for depth")
            rospy.wait_for_message(self.dep_topic, Image)
        self.start_time = time.time()

    def callback_rgb(self, image):
        self.rgb = CvBridge().imgmsg_to_cv2(image, "bgr8")
        self.start_time = time.time()

    def callback_dep(self, image):
        self.dep = CvBridge().imgmsg_to_cv2(image, "passthrough")
        self.start_time = time.time()

    def read(self, channel: Literal["rgb", "depth"] = "rgb", timeout=3):
        if time.time() - self.start_time > timeout:
            return (False, None)

        if channel == "rgb":
            return (self.rgb is not None, self.rgb)
        return (self.dep is not None, self.dep)