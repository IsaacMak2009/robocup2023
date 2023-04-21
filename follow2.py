#!/usr/bin/env python3
import rospy
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from libs.pidlib import PID
from libs.astra import RosAstraCamera as Camera

from pcms.openvino_models import Yolov8
from Mlogger.mlogger import Logger

logger = Logger()

def find_human(frame):
    try:
        result = model.forward(frame) # data: x1, y1, x2, y2, conf, type
        data = result[0]["det"].tolist()
    except:
        return (-1,-1)
    #print(data)
    cx, cy = 0, 0
    for i in data:
        if i[5]==0:
            cx = (i[0] + i[2]) / 2
            cx = int(cx)
            cy = (i[1] + i[3]) / 2
            cy = int(cy)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            cv2.line(frame, (cx, cy), (w//2, h//2), (255, 0, 0), 3)
            return (cx, cy)
    return (-1, -1)


if __name__ == "__main__":
    rospy.init_node("follower")
    logger.info("follower node start!")

    logger.debug("Loading camera...")
    cam = Camera()
    logger.debug("Loading yolov8...")
    model = Yolov8()

    logger.debug("Loading turtlebot")
    publisher = rospy.Publisher("/mobile_base/commands/velocity", Twist)
    rospy.sleep(1)
    logger.info("ready to start!")
    
    pid = PID(0.6, 0.25, 0.25)
    # MAIN LOOP
    rospy.sleep(1)
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        ret, frame = cam.read()
        h, w = frame.shape[:2]
        cx, cy = find_human(frame)
        cx = int(cx)
        cy = int(cy)
        if cx != -1 and cy != -1:
            ret, depth = cam.read("depth")
            h, w = frame.shape[:2]
            power = pid.control(target=cx, curr=w/2) / (w/2)*(-1.75)
            print(power)
            data = Twist()
            data.angular.z = power
            if abs(cx-w/2) < 125 and depth[cy][cx] > 500:
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
