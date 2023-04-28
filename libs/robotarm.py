import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.srv import SetKinematicsPose, SetKinematicsPoseRequest

import time

def set_gripper(angle, t):
    service_name = "/goal_tool_control"
    rospy.wait_for_service(service_name)
    
    try:
        service = rospy.ServiceProxy(service_name, SetJointPosition)
        
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = [angle]
        request.path_time = t
        
        response = service(request)
        return response
    except Exception as e:
        rospy.loginfo("%s" % e)
        return False

def move_to(x, y, z, t):
    service_name = "/goal_task_space_path_position_only"
    rospy.wait_for_service(service_name)
    
    try:
        service = rospy.ServiceProxy(service_name, SetKinematicsPose)
        
        request = SetKinematicsPoseRequest()
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = x
        request.kinematics_pose.pose.position.y = y
        request.kinematics_pose.pose.position.z = z
        request.path_time = t
        
        response = service(request)
        return response
    except Exception as e:
        rospy.loginfo("%s" % e)
        return False

def set_joints(joint1, joint2, joint3, joint4, t):
    service_name = "/goal_joint_space_path"
    rospy.wait_for_service(service_name)
    
    try:
        service = rospy.ServiceProxy(service_name, SetJointPosition)
        
        request = SetJointPositionRequest()
        request.joint_position.joint_name = ["joint1", "joint2", "joint3", "joint4"]
        request.joint_position.position = [joint1, joint2, joint3, joint4]
        request.path_time = t
        
        response = service(request)
        return response
    except Exception as e:
        rospy.loginfo("%s" % e)
        return False

def open_gripper(t):
    return set_gripper(0.005, t)

def close_gripper(t):
    return set_gripper(-0.003, t)