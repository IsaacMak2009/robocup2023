#!/usr/bin/env python3
import rospy
import cv2
import time
import sys

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from libs.pidlib import PID
from libs.astra import RosAstraCamera as Camera
from libs.RobotChassis import RobotChassis
from libs.robotarm import open_gripper

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
    shortest_d = float('inf')
    target = None
    for i in data:
        if i[5]==0:
            cx = (i[0] + i[2]) / 2
            cx = int(cx)
            cy = (i[1] + i[3]) / 2
            cy = int(cy)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            if cam.get_real_xyz(cx, cy).length() < shortest_d:
                shortest_d = cam.get_real_xyz(cx, cy).length()
                target = (cx, cy)
                break

    if target is None:
        return (-1, -1)
    else:
        cv2.line(frame, target, (w//2, h//2), (255, 0, 0), 3)
        return target


if __name__ == "__main__":
    rospy.init_node("follower")
    logger.info("follower node start!")

    logger.debug("Loading camera...")
    cam = Camera("/camera2/rgb/image_raw", "/camera2/depth/image_raw")

    logger.debug("Loading yolov8...")
    model = Yolov8(device_name="GPU")

    logger.debug("Loading robotchassis")
    chassis = RobotChassis()

    logger.debug("Loading turtlebot")
    publisher = rospy.Publisher("/cmd_vel", Twist)
    rospy.sleep(1)

    logger.info("ready to start!")
    
    pid = PID(0.6, 0.25, 0.25)
    # MAIN LOOP
    rospy.sleep(1)
    cnt = 0
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()

        ret, frame = cam.read()
        h, w = frame.shape[:2]
        cx, cy = find_human(frame)
        cx = int(cx)
        cy = int(cy)

        # if the idiot stayed for 3s or longer 
        if cnt >= 3:
            #open_gripper(3)
            
            logger.info(f"Start go back to home")
            curr_P = chassis.get_current_pose()
            logger.info(f"From {curr_P=}")
            target=[-1.03, 0, 0]
            logger.info(f"To {target=}")
            chassis.move_to(*target)
            while chassis.status_code != 3:
                time.sleep(0.1)
                logger.debug(f"{chassis.status_text}")
            sys.exit()

        # follow
        if cx != -1 and cy != -1:
            ret, depth = cam.read("depth")
            h, w = frame.shape[:2]
            power = pid.control(target=cx, curr=w/2) / (w/2)*(-1.75)
            data = Twist()
            data.angular.z = power
            if abs(cx-w/2) < 125 and depth[cy][cx] > 1000:
                data.linear.x = 0.2
            if depth[cy][cx] < 1000 and abs(cx-w/2) < 125:
                cnt += 1/18 # rate 20f/s
                logger.info(f"Staying... {cnt}")
            publisher.publish(data)

        # show image
        cv2.imshow("frame", frame)
        key_code = cv2.waitKey(5)
        if key_code in [27, ord('q')]:
            break

    rospy.loginfo("follower node end!")
