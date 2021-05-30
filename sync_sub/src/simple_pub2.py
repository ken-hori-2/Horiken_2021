#!/usr/bin/env python
# -*- coding: utf-8 -*-
# license removed for brevity
import rospy
from sync_sub.msg import some_position

def talker():
    #チュートリアルのコードからメッセージの型だけ変えてある
    pub = rospy.Publisher('chatter2', some_position, queue_size=10)
    rospy.init_node('talker2', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = some_position()
        #rospyでheaderにタイムスタンプを入れる
        msg.header.stamp = rospy.Time.now()
        #適当なデータ
        msg.position = [4,5,6]
        pub.publish(msg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
