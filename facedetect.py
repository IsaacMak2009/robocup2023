import rospy
from mr_voice.msg import Voice
from std_msgs.msg import String
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
def read(path):
    with open(path) as f:
        return np.array(f.read().split(','), dtype=np.float32)

queue_vec = None
faces = []
with open("faces/faces.txt") as f:
    for name in f.readlines():
        print(name)
        faces.append((name, read(f"faces/{name}.csv")))

print("loading camera")
cam = Camera()

print("loading model...")
face_detection = FaceDetection()
face_identification = FaceReidentification()

print("loading respeaker")

def on_voice(msg: Voice):
    global name, asking
    if 'name' in msg.text:
        name = msg.text.split(' ')[-1]

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
        face_frame = frame[box[1]:box[3], box[0]:box[2]]
        vec = face_identification.forward(face_frame)

        found = False
        for face in faces:
            dist = face_identification.compare(vec, face[1])
            if dist < 0.5:
                cv2.rectangle(frame, box[:2], box[2:], (255, 255, 0), 3)
                cv2.putText(frame, name, box[:2], cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
                found = True
                break
        if found: continue

        if queue_vec is not None and face_identification.compare(vec, queue_vec) < 0.5:
            cv2.rectangle(frame, box[:2], box[2:], (0, 0, 255), 3)


        # now process unknown face
        if asking: # already asking
            if name: # found
                print(name)
                sayer.publish(f"{name}, your name is saved")
                faces.append((name, queue_vec))
                queue_vec = None
                asking = False
        else:
                    
            # ask
            asking = True
            sayer.publish("What is your name")
            name = None
            queue_vec = vec


    cv2.imshow("frame", frame)
    if cv2.waitKey(1) == 27 or _safe_exit:
        break
