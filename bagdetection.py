import rospy
from mr_voice.msg import Voice
from libs.astra import RosAstraCamera as Camera
from libs.math3d import *
from Mlogger.mlogger import Logger
from pcms.openvino_models import Yolov8
import cv2
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from libs.pidlib import PID
from libs.robotarm import set_joints, open_gripper, close_gripper

select = "N" # L, N, R
logger = Logger()
frame, depth = None, None

def on_voice(msg: Voice):
    logger.debug(msg.text)
    global select, depth, frame
    if "reset" in msg.text:
        select = "N"
        logger.info("N")
        return
    elif "left" in msg.text or "black" in msg.text:
        select = "L"
        logger.info("L")
        return
    elif "right" in msg.text or "white" in msg.text:
        select = "R"
        logger.info("R")
        return

    if ("take" in msg.text or "cake" in msg.text or "pick" in msg.text or "kick" in msg.text or "back" in msg.text) and bx is not None:
        logger.info("Taking the bag...")
        # take the bag
        size = vec2(frame.shape[0], frame.shape[1])
        bx_vec = (
            int((bx[0]+bx[2])/2), # middle x
            int((bx[1]+bx[3])/2)
        )
        xyd = vec3(
            (bx[0]+bx[2])/2,
            (bx[1]+bx[3])/2,
            depth[bx_vec[1], bx_vec[0]],
        )
        xyz = get_real_xyz(xyd, size)
        if select == "L":
            dname = "left"
        if select == "R":
            dname = "right"
        publisher.publish(f"I am going to take the {dname} one now")

        # turn to bag
        distance = xyz.length()
        angle = -math.atan2(xyz.x, xyz.z)
        logger.debug("xyz: "+str(xyz))
        logger.debug("angle: "+str(angle))
        data = Twist()
        data.angular.z = angle * 1.1
        turtlebot.publish(data)
        rospy.sleep(2)

        # move to the bag
        logger.debug("distance: "+str(distance))
        '''for i in range(int(distance/2)):
            data = Twist()
            data.linear.x = 0.175
            turtlebot.publish(data)
            rospy.Rate(20).sleep()'''
        pid = PID(0.6, 0.1, 0.25)
        while True:
            ret, dep = cam.read("depth")
            dx = dep[:, dep.shape[1] // 2].copy().reshape((dep.shape[0],))
            dx[dx==0] = 9999
            d = np.min(dx)

            ret, dep = cam.read("depth")
            dx = dep[:, dep.shape[1] // 2].copy().reshape((dep.shape[0],))
            dx[dx==0] = 9999
            d = (d+np.min(dx))/2

            if d<=468 or d==9999: break
            power = -pid.control(target=100, curr=d)/d/3
            logger.info(f"d: {d}, power: {power}")
            data = Twist()
            data.linear.x = power
            turtlebot.publish(data)
            rospy.Rate(20).sleep()

        data = Twist()
        data.linear.x = 0.15
        for i in range(15):
            turtlebot.publish(data)
            rospy.Rate(20).sleep()

        # TODO: use the arm to get the bag
        target_data = (0.051, -0.400, 0.184, 0.983)
        home_data = (0.05, -1.150, 0.28, 0.18)

        open_gripper(3)
        time.sleep(3)

        set_joints(*target_data, 3)
        time.sleep(3)

        close_gripper(3)
        time.sleep(3)

        set_joints(*home_data, 3)
        time.sleep(3)
        return
    
    publisher.publish("Can you speak again?")


def select_bag(b1, b2):
    if select == "N":
        return None
    if select == "L":
        return min(b1, b2, key=lambda box: (box[0]+box[3])/2)
    if select == "R":
        return max(b1, b2, key=lambda box: (box[0]+box[3])/2)


def debuger_callback(msg: String):
    m = Voice()
    m.text = msg.data
    on_voice(m)

if __name__=='__main__':
    rospy.init_node("bag_take")
    rospy.loginfo("bag_take node started")

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

    logger.info("loading camera")
    cam = Camera(use_pro=True)

    logger.info("loading model")
    start_time = time.time()
    model = Yolov8(model_name="v6best2", device_name="GPU")
    model.classes = ["bag"]
    logger.debug(f"used {time.time()-start_time:.3f}s to load the model")
    
    rospy.Subscriber("/voice/text", Voice, on_voice)
    publisher = rospy.Publisher("/speaker/say", String, queue_size=10)
    turtlebot = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    debuger = rospy.Subscriber("~/cmd", String, debuger_callback)

    asked = False

    publisher.publish("Hello, my name is neiga")

    while True:
        if _safe_exit: break
        rospy.Rate(15).sleep()
        ret, frame = cam.read()
        ret, depth = cam.read(channel="depth")
        if ret and cv2.waitKey(1) != ord('q'):
            result = model.forward(frame)
            try:
                boxes: list = result[0]["det"].tolist()
            except:
                continue # no object detect
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
                    asked = False
                else:
                    if not asked:
                        publisher.publish("Which bag do you want to take?")
                        asked = True

            cv2.imshow("frame", frame)


        else:
            break
