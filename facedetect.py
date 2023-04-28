import rospy
from mr_voice.msg import Voice
from std_msgs.msg import String
from Mlogger.mlogger import Logger
from libs.astra import RosAstraCamera as Camera
import cv2
import json
from pcms.openvino_models import FaceReidentification, FaceDetection, AgeGenderRecognition
import numpy as np

# https://stackoverflow.com/a/1305682
class obj(object):
    def __init__(self, d):
        for k, v in d.items():
            if isinstance(k, (list, tuple)):
                setattr(self, k, [obj(x) if isinstance(x, dict) else x for x in v])
            else:
                setattr(self, k, obj(v) if isinstance(v, dict) else v)

def identity(tface, known):
    min_idx = -1
    min_i = 9999
    for idx, face in enumerate(known):
        distance = face_identification.compare(face, tface)
        if distance < min_i:
            min_idx = idx
            min_i = distance
    return (min_i, min_idx)


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

    unknown = []
    known = []
    known_name = []
    
    cam = Camera(rgb="/camera2/rgb/image_raw", dep=None)

    logger.debug("loading model...")
    face_detection = FaceDetection()
    face_identification = FaceReidentification()

    def on_voice(msg: Voice):
        global xname, asking_name
        xname = msg.text
        for word in ["name", "is", "my", "hello","i","am"]:
            xname = xname.replace(word, '')

        if asking_name:
            known.append(unknown.pop(0))
            known_name.append(xname)
            asking_name = False

    rospy.Subscriber("/voice/text", Voice, on_voice)
    sayer = rospy.Publisher("/speaker/say", String, queue_size=10)
    asking_name = False
    xname = ""
    idx = 0

    ret, frame = cam.read()
    face = face_detection.forward(frame)
    for box in face:
        cv2.rectangle(frame, box[:2], box[2:], (255, 0, 0), 3)
        face_vec = face_identification.forward(frame[box[1]:box[3], box[0]:box[2]])
        conf, name = identity(face_vec)
        if conf < 0.6:
            unknown.append(face_vec)


    while True:
        rospy.Rate(30).sleep()
        ret, frame = cam.read()
        face = face_detection.forward(frame)
        for box in face:
            cv2.rectangle(frame, box[:2], box[2:], (255, 0, 0), 3)
            face_vec = face_identification.forward(frame[box[1]:box[3], box[0]:box[2]])
            conf, idx_face = identity(face_vec, known)
            if conf < 0.6:
                # known
                # TODO: draw the idiot face
                cv2.putText(known_name[idx_face], box[:2])
                continue
            conf, name = identity(face_vec, unknown)
            if name == 0 and not asking_name:
                asking_name = True
                xname = None
                sayer.publish("What is your name")
                # TODO: turn to the idiot
                
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) == 27 or _safe_exit:
            break
