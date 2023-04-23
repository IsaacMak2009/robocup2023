import rospy
from mr_voice.msg import Voice
from std_msgs.msg import String
from Mlogger.mlogger import Logger
from libs.astra import RosAstraCamera as Camera
import cv2
import json
from pcms.openvino_models import FaceReidentification, FaceDetection
import numpy as np

if __name__=='__main__':
    logger = Logger()
    rospy.init_node("face_detect")
    logger.info("face_detect node start!")

    logger.debug("staring safe_exit")
    use_safe_exit = True
    _safe_exit = False
    if use_safe_exit:
        import signal
        def safe_exit(*args, **kwargs):
            global _safe_exit
            logger.warn("SAFE EXIT!")
            _safe_exit=True
        signal.signal(signal.SIGINT, safe_exit)
        signal.signal(signal.SIGTERM, safe_exit)

    logger.debug("loading faces...")
    known_faces = None
    with open("faces/faces.json") as f:
        known_faces = json.load(f)

    logger.debug("loading camera")
    cam = Camera(dep=None)

    logger.debug("loading model...")
    face_detection = FaceDetection()
    face_identification = FaceReidentification()

    logger.debug("loading respeaker")

    def on_voice(msg: Voice):
        global name, asking
        xname = msg.text
        for word in ["name", "is", "my", "hello","i","am"]:
            xname = xname.replace(word, '')
        return xname

    rospy.Subscriber("/voice/text", Voice, on_voice)
    sayer = rospy.Publisher("/speaker/say", String, queue_size=10)
    asking = False
    name = ""

    while True:
        rospy.Rate(30).sleep()
        ret, frame = cam.read()
        face = face_detection.forward(frame)
        for box in face:
            cv2.rectangle(frame, box[:2], box[2:], (255, 0, 0), 3)
            


        cv2.imshow("frame", frame)
        if cv2.waitKey(1) == 27 or _safe_exit:
            break
