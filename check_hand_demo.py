from libs.astra import RosAstraCamera as Camera
from pcms.openvino_models import HumanPoseEstimation
from Mlogger.mlogger import Logger
import cv2
import rospy

if __name__ == "__main__":
    rospy.init_node("trash")
    logger = Logger()
    dnn_human_pose = HumanPoseEstimation()
    cam = Camera()

    while not rospy.is_shutdown():
        image = cam.read()
        frame = image.copy()
        poses = dnn_human_pose.forward(image)
        frame = dnn_human_pose.draw_poses(frame, poses, 0.1)
        for pose in poses:
            for i, p in enumerate(pose):
                x, y, c = map(int, p)
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)
            pt1 = pose[8]
            pt2 = pose[10]
            k = (pt1[1] - pt2[1])/(pt1[0] - pt2[0])
            if k>0:
                logger.info("Left") # left
            else:
                logger.info("right") # right