#!/usr/bin/env python

import rospy
import message_filters
from sync_sub.msg import some_position
from ros_lecture_msgs.msg import Custom2

def callback(msg1, msg2,msg3,msg4):
  # do something
  position1 = msg1.position
  position2 = msg2.position
  position3 = msg3.position
  position4 = msg4.position
  out = [position1, position2, position3, position4]
  print(out)


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    sub1 = message_filters.Subscriber('chatter1', some_position)
    sub2 = message_filters.Subscriber('chatter2', some_position)
    sub3 = message_filters.Subscriber('chatter3', some_position)
    sub4 = message_filters.Subscriber('chatter4', some_position)

    fps = 10.
    delay = 1/fps

    ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2,sub3,sub4], 10, delay)
    ts.registerCallback(callback)
    rospy.spin()
