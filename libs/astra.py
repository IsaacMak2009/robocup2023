import rospy
import time
import cv2
from cv_bridge import CvBridge
from typing import *
from sensor_msgs.msg import Image
from Mlogger.mlogger import Logger
from libs.math3d import vec2, vec3, get_real_xyz


class RosAstraCamera:
    def __init__(self, rgb: Optional[str] = "/camera/rgb/image_raw", dep: Optional[str] = "/camera/depth/image_raw", use_pro: bool = False):
        self.rgb_topic = rgb
        self.dep_topic = dep
        self.rgb = None
        self.dep = None
        self.logger = Logger()
        self.use_pro = use_pro
        if rgb or rgb is not None:
            rospy.Subscriber(self.rgb_topic, Image, self.callback_rgb)
            self.logger.info("[astra.py] waiting for rgb")
            rospy.wait_for_message(self.rgb_topic, Image)

        if dep or dep is not None:
            rospy.Subscriber(self.dep_topic, Image, self.callback_dep)
            self.logger.info("[astra.py] waiting for depth")
            rospy.wait_for_message(self.dep_topic, Image)
        self.start_time = time.time()

    def callback_rgb(self, image):
        self.rgb = CvBridge().imgmsg_to_cv2(image, "bgr8")
        if self.use_pro:
            self.rgb = cv2.resize(self.rgb, (640, 480))
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
    
    def get_real_xyz(self, x, y) -> vec3:
        d = self.dep[y, x]
        size = vec2(640, 480)
        return get_real_xyz(vec3(x, y, d), size)