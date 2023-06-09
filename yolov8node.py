#!/usr/bin/env python3
import rospy
import cv2
from libs.astra import RosAstraCamera as Camera
from pcms.openvino_models import Yolov8
from Mlogger.mlogger import logger

if __name__ == '__main__':
    rospy.init_node("yolov8_node")
    logger.info("yolov8_node started!")

    logger.info("loading camera")
    cam = Camera(dep=None)

    logger.info("loading model")
    model = Yolov8(model_name="v6best2", device_name="GPU")
    model.classes = ["bag"]

    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        ret, frame = cam.read()
        result = model.forward(frame)
        for obj in result[0]["det"]:
            # obj: [x1, y1, x2, y2, score, label_id]
            obj = obj.detach().tolist()
            #print(obj)
            #if obj[4] <= 0.65:
            #    continue
            cv2.rectangle(frame, (int(obj[0]),int(obj[1])), (int(obj[2]),int(obj[3])), (0, 255, 0), 1)
        cv2.imshow("result", frame)
        if cv2.waitKey(5) & 0xFF == 27: break
