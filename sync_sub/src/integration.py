#!/usr/bin/env python

import rospy
import message_filters
from sync_sub.msg import some_position
from sync_sub.msg import some_position2

from geometry_msgs.msg import Twist
from ros_lecture_msgs.msg import Custom2
from ros_beginner.msg import Kpub

a=0

def callback(msg1, msg2):
  # do something
  position1 = msg1.Class
  position2 = msg2.position

  
  out = [position1, position2[0],position2[1]]
  print(out)
  #out = [Class,blue_area,red_area]
  msg = Kpub()
  msg.position = position2
  msg.Class = position1
  rospy.loginfo(msg)
  pub.publish(msg)


  if position1 == "traffic light" and position2[1] > 500:
            rospy.loginfo("test_stop!")
            vel.linear.x = -1
            vel_pub.publish(vel)


  #          a=7
            #print(a)
            #return 7

  #if position1 == "traffic light" :
  #          rospy.loginfo("test_stop!")

  #          if position2[1] > 500:
  #                rospy.loginfo("STOP!!!!")
  #                vel.linear.x = 0
  #                vel_pub.publish(vel)          


  if position1 == "traffic light" and position2[0] > 500:
            rospy.loginfo("test_Advance!!!!!")
            vel.linear.x = 1
            vel_pub.publish(vel)
            


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    sub1 = message_filters.Subscriber('chatter1', some_position2)
    sub2 = message_filters.Subscriber('chatter2', some_position)

    

    vel_pub = rospy.Publisher('/dtw_robot1/diff_drive_controller/cmd_vel', Twist, queue_size=10)
    vel = Twist()
    
    pub = rospy.Publisher('chatter5', Kpub, queue_size=10)
    #pub2 = rospy.Publisher('chatter6', some_position, queue_size=10)

    fps = 10.
    delay = 1/fps

    ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 10, delay)
    ts.registerCallback(callback)

    #print(a)
    #print(1)
    #if a == 7:
    #  print(a)
    #  vel.linear.x = 1
    #  vel_pub.publish(vel)
    
    #callback_lambda = lambda x: callback(x, vel, vel_pub)
    rospy.spin()
