#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from libs.pidlib import PID
from std_msgs.msg import Int32MultiArray
import cv2, time

from ultralytics import YOLO


def callback_image(msg):
    global _image
    _image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

def callback_depth(msg):
    global _depth
    _depth = CvBridge().imgmsg_to_cv2(msg, "passthrough")

def rate(n):
    time.sleep(1/n)
    rospy.Rate(n).sleep()

def find_human(frame):
    result = model(frame) # data: x1, y1, x2, y2, conf, type
    data = result[0].boxes.data.detach().tolist()
    print(data)
    cv2.imshow("yolov8", result[0].plot())
    cx, cy = 0, 0
    for i in data:
        if i[5]==0:
            cx = (i[0] + i[2]) / 2
            cx = int(cx)
            cy = (i[1] + i[3]) / 2
            cy = int(cy)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.line(frame, (cx, cy), (w//2, h//2), (255, 0, 0), 1)
            return (cx, cy)
    return (-1, -1)


if __name__ == "__main__":
    rospy.init_node("follower")
    rospy.loginfo("follower node start!")

    # ROS Topics
    _image = None
    _depth = None
    moved = False
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback_image)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
    rospy.wait_for_message("/camera/rgb/image_raw", Image)
    rospy.wait_for_message("/camera/depth/image_raw", Image)
    rospy.loginfo("camera ok")

    publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist)
    rospy.sleep(1)
    rospy.loginfo("turtlebot ok")
    
    model = YOLO("/home/pcms/yolov8n_openvino_model")
    rospy.sleep(1)
    rospy.loginfo("model ok")
    
    pid = PID(0.6, 0.25, 0.25)
    # MAIN LOOP
    rospy.sleep(1)
    while not rospy.is_shutdown():
        rate(20)
        frame = _image.copy()
        h, w = frame.shape[:2]
        cx, cy = find_human(frame)
        cx = int(cx)
        cy = int(cy)
        if cx != -1 and cy != -1:
            h, w = frame.shape[:2]
            power = pid.control(target=cx, curr=w/2) / (w/2)*(-1.75)
            print(power)
            data = Twist()
            data.angular.z = power
            if abs(cx-w/2) < 125 and _depth[cy][cx] > 500:
                data.linear.x = 0.20
                moved = 3
            elif moved != 0:
                data.linear.x = 0.05 * moved
                moved -= 1
            publisher.publish(data)
        # show image
        cv2.imshow("frame", frame)
        key_code = cv2.waitKey(5)
        if key_code in [27, ord('q')]:
            break

    rospy.loginfo("follower node end!")
