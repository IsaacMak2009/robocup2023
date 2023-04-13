#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Int32MultiArray
import numpy as np
import cv2
import time,sys

class Yolov8:
    def __init__(self, model_name="/home/pcms/bagdetection_openvino_model"):
        self.image = None
        self.model = YOLO(model_name)
        self.rate = 100
        self.detecting = False
        self.saved_time = time.time()
        self.publisher = rospy.Publisher("~raw_data", Int32MultiArray)
        
        rospy.loginfo("waiting for camera")
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback_image)
        rospy.wait_for_message("/camera/rgb/image_raw", Image)
        rospy.loginfo("camera ok")
        
        
    def callback_image(self, msg):
        if time.time() - self.saved_time > 1/self.rate and self.detecting == False:
            self.detecting = True
            self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.result = self.model.predict(self.image)
            
            self.frame = self.result[0].plot()
            fps = 1/(time.time()-self.saved_time)
            self.frame = cv2.putText(self.frame, f"FPS: {fps:.2f}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 4, cv2.LINE_AA)
            cv2.imshow("frame", self.frame)
            if cv2.waitKey(1) in [27, ord('q')]:
                sys.exit()
            
            data = self.result[0].boxes.data.tolist()
            raw_data = [len(data)]
            # header: objects (1 int)
            for obj in data:
                for i,j in enumerate(obj):
                    if i!=4:
                        raw_data.append(round(j))
            msg = Int32MultiArray()
            msg.data = raw_data
            self.publisher.publish(msg)
            self.saved_time = time.time()
            self.detecting = False
            

if __name__ == "__main__":
    rospy.init_node("yolov8")
    rospy.loginfo("yolov8 node start!")
    Yolov8()
    rospy.spin()
