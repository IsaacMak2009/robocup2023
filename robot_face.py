import cv2
import time
import rospy
import random
from mr_voice.msg import Voice
from std_msgs.msg import String
import os

path = "QnA.txt"
if not os.path.exists(path):
    print("file not exists!")
    exit(1)

with open(path, "r") as f:
    lines = f.readlines()

q, a, cmd = [], [], []
for line in lines:
    s = line.strip()
    if len(s) == 0: continue
    q.append(s.split('/')[1])
    a.append(s.split('/')[2])
    if len(s.split('/'))==4:
        cmd.append(s.split('/')[-1])
    else:
        cmd.append('')
print(cmd)
    

_text = ''
def callback_text(msg):
    global _text
    _text = msg.text
    print(msg.text)
   



video_speak = cv2.VideoCapture("./videos/robot1.mp4")
video_norms = cv2.VideoCapture("./videos/robot2.mp4")
speaking = True
frame_cnt = 0
saved_frame = 0

rospy.init_node("speak_robot")
speaker = rospy.Publisher("/speaker/say", String, queue_size=10)
rospy.Subscriber("/voice/text", Voice, callback_text)
#rospy.wait_for_message("/voice/text", String)

last_text = ""
while(video_speak.isOpened() and video_norms.isOpened() and not rospy.is_shutdown()):
    speaking = rospy.get_param("/speaker/is_saying")
    if speaking:
        ret, frame = video_speak.read()
        if not ret:
           video_speak.set(cv2.CAP_PROP_POS_FRAMES, 0)
           continue
    else:
        ret, frame = video_norms.read()
        if not ret:
           video_norms.set(cv2.CAP_PROP_POS_FRAMES, 0)
           continue
    cv2.imshow("Image", frame)
    
    time.sleep(1/30)
    rospy.Rate(30).sleep()
    if not speaking:
        # speak
        if _text != last_text:
            k1 = _text.lower().split(" ")
            best_i, best_cnt = [], 0
            for i in range(len(q)):
                k2 = q[i][:-1].lower().split(" ")
                cnt = len([k for k in k1 if k in k2])
                if cnt > best_cnt:
                    best_i, best_cnt = [i], cnt
                if cnt == best_cnt:
                    best_i.append(i)
            ai = random.choice(best_i)
            respone = a[ai]
            speaker.publish(respone)
            rospy.loginfo(f"speaking: {respone}")
            os.system(cmd[ai])
            rospy.loginfo(f"cmd: {cmd[ai]}")
            last_text = _text
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    
video_speak.release()
video_norms.release()
cv2.destroyAllWindows()

