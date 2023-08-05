#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import math
import sys
import traceback
import time

def main():

    global x,y,theta
    rospy.init_node('main_program', anonymous=True)
    pen_pub = rospy.Publisher('move_pen', Int32, queue_size=10) 
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    time.sleep(1)
    
    while not rospy.is_shutdown():

        
        if sys.argv[1]=='sq':
            pen_pub.publish(1) 
            current_time=time.time()
            vel_msg.linear.x = 0.0005
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            while time.time()-current_time<2:
                vel_pub.publish(vel_msg)
                print("Go forward!!")
            time.sleep(1)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_pub.publish(vel_msg)
            time.sleep(1)
            current_time=time.time()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0.0005
            while time.time()-current_time<2:
                vel_pub.publish(vel_msg)
                print("Go right!!")
            time.sleep(1)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_pub.publish(vel_msg)
            time.sleep(1)
            current_time=time.time()
            vel_msg.linear.x = -0.0002
            vel_msg.linear.y = 0
            while time.time()-current_time<2:
                vel_pub.publish(vel_msg)
                print("Go backward!!")
            time.sleep(1)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_pub.publish(vel_msg)
            time.sleep(1)
            current_time=time.time()
            vel_msg.linear.x = 0
            vel_msg.linear.y = -0.0005
            while time.time()-current_time<2:
                vel_pub.publish(vel_msg)
                print("Go left!!")
            time.sleep(1)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_pub.publish(vel_msg)
            time.sleep(1)

        elif  sys.argv[1]=='ci':
            pen_pub.publish(2) 
            current_time=time.time()
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0.05
            while time.time()-current_time<2:
                vel_pub.publish(vel_msg)
                print("Go in circle!!")
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            vel_pub.publish(vel_msg)

        elif  sys.argv[1]=='line':
            pen_pub.publish(2) 
            current_time=time.time()
            vel_msg.linear.x = 0.05
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            while time.time()-current_time<3:
                vel_pub.publish(vel_msg)
                print("Go forward!!")
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_pub.publish(vel_msg)
            print("Stop!!")
            time.sleep(2)
            current_time=time.time()
            vel_msg.linear.x = -0.0003
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            while time.time()-current_time<3:
                vel_pub.publish(vel_msg)
                print("Go backward!!")
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_pub.publish(vel_msg)
            print("Stop!!")
        time.sleep(1)
        pen_pub.publish(2) 
       
        rospy.spin()  

   
if __name__ == "__main__":
    try:
        main()
    except:
        traceback.print_exc(file=sys.stdout)
        sys.exit()

    finally:
        print('Success')

