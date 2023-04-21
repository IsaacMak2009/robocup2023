import rospy
from mr_voice.msg import Voice
from libs.astra import RosAstraCamera as Camera
from ultralytics import YOLO
import cv2

select = "N" # L, N, R
def on_voice(msg: Voice):
    print(msg.text)
    global select
    if "reset" in msg.text:
        select = "N"
        rospy.loginfo("N")
    elif "left" in msg.text:
        select = "L"
        rospy.loginfo("L")
    elif "right" in msg.text:
        select = "R"
        rospy.loginfo("R")

def select_bag(b1, b2):
    if select == "N":
        return None
    if select == "L":
        return min(b1, b2, key=lambda box: (box[0]+box[3])/2)
    if select == "R":
        return max(b1, b2, key=lambda box: (box[0]+box[3])/2)


if __name__=='__main__':
    rospy.init_node("bag_take")
    rospy.loginfo("bag_take node started")
    model = YOLO("/home/pcms/bagdetection_openvino_model")
    cam = Camera()
    rospy.Subscriber("/voice/text", Voice, on_voice)

    while True:
        rospy.Rate(20).sleep()
        ret, frame = cam.read()
        if ret and cv2.waitKey(1) != ord('q'):
            result = model(frame, verbose=False)
            boxes: list = result[0].boxes.data.tolist()
            for box in boxes:
                bbox = list(map(int, box))
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), int(3*box[-2]))

            if len(boxes) < 2:
                pass
            else:
                boxes.sort(key=lambda bbox: bbox[-2], reverse=True) # sort by conf
                bx1 = boxes[0]
                bx2 = boxes[1]
                bx = select_bag(bx1, bx2)
                if bx:
                    bx = list(map(int, bx))
                    cv2.rectangle(frame, (bx[0], bx[1]), (bx[2], bx[3]), (0, 0, 255), 3)

            cv2.imshow("frame", frame)


        else:
            break
