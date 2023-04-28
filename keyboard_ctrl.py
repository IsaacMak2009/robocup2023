import pygame
import rospy
from geometry_msgs.msg import Twist

rospy.init_node("controller")
publisher = rospy.Publisher("/cmd_vel", Twist)
rospy.sleep(1)
rospy.loginfo("turtlebot ok")
pygame.init()
window = pygame.display.set_mode((300, 300))
clock = pygame.time.Clock()

run = True
speed = 0
addspeed = 0.025
maxspeed = 0.2
esp = 0.05
def iszero(f):
    return abs(f) < esp
keys = dict()
while run:
    clock.tick(10)
    rospy.Rate(10).sleep()
    flag = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False
        if event.type == pygame.KEYDOWN:
            keys[event.key] = True
            print(pygame.key.name(event.key))
        if event.type == pygame.KEYUP:
            keys[event.key] = False
            
    data = Twist()
    if keys.get(pygame.K_w):
        if speed<0:
            speed = addspeed
        else:
            speed = min(maxspeed, speed+addspeed) 
        flag=False
        data.linear.x = speed
        print("move forward", speed)
        
    if keys.get(pygame.K_a):
        data.angular.z = 1.1
        publisher.publish(data)
        print("turn left 1.1")
        
    if keys.get(pygame.K_s):
        if speed>0:
            speed = -addspeed
        else:
            speed = max(-maxspeed, speed-addspeed) 
        flag=False
        data.linear.x = speed
        print("move forward", speed)
        
    if keys.get(pygame.K_d):
        flag = False
        data.angular.z = -1.1
        print("turn right 1.1")
        
    if not flag:
        publisher.publish(data)
    if flag and not(iszero(speed)):
        if speed>0:
            speed -= addspeed * 0.55
            speed = max(0, speed)
        else:
            speed += addspeed * 0.55
            speed = min(0, speed)
        print(speed)
        data.linear.x = speed
        publisher.publish(data)
        
        

pygame.quit()
