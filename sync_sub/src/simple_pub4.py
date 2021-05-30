#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from sync_sub.msg import some_position
from ros_lecture_msgs.msg import Custom2


def talker():
    #チュートリアルのコードからメッセージの型だけ変えてある
    #pub = rospy.Publisher('ros_lecture_msgs', Custom2, queue_size=10)
    
    pub = rospy.Publisher('chatter4',some_position, queue_size=10)
    
    rospy.init_node('talker4', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #msg = Custom2()

        msg = some_position()

        #rospyでheaderにタイムスタンプを入れる
        msg.header.stamp = rospy.Time.now()
        #適当なデータ
        msg.position = [7.5,8.5,9.5]
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
