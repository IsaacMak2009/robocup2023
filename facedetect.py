import rospy
rospy.init_node("aaaaa")

print("loading cam")
from libs.astra import RosAstraCamera as Camera
import cv2

print("loading openvino...")
from pcms.openvino_models import FaceReidentification, FaceDetection, np                                      

print("staring safe_exit")
use_safe_exit = True
_safe_exit = False
if use_safe_exit:
    import signal
    def safe_exit(*args, **kwargs):
        global _safe_exit
        print("SAFE EXIT!")
        _safe_exit=True
    signal.signal(signal.SIGINT, safe_exit)
    signal.signal(signal.SIGTERM, safe_exit)

print("loading face...")
face1 = []
with open("faces/IsaacPro.csv", 'r') as f:
    face1 = f.read().split(',')
face1 = np.array(face1, dtype=np.float32)
print(face1.shape)

cam = Camera()
print("loading model...")
face_detection = FaceDetection()
face_identification = FaceReidentification()

while True:
    rospy.Rate(30).sleep()
    ret, frame = cam.read()
    face = face_detection.forward(frame)
    for box in face:
        cv2.rectangle(frame, box[:2], box[2:], (255, 0, 0), 3)
        face_frame = frame[box[1]:box[3], box[0]:box[2]]
        vec = face_identification.forward(face_frame)
        dist = face_identification.compare(vec, face1)
        if dist < 0.5:
            cv2.rectangle(frame, box[:2], box[2:], (255, 255, 0), 3)
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) == 27 or _safe_exit:
        break
