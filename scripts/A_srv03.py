#! /usr/bin/env python

import rospy
from ros_beginner.srv import Kyolo,KyoloResponse

from sync_sub.msg import some_position
from sync_sub.msg import some_position2
import math

from geometry_msgs.msg import Twist
import time


class Integration(object):
    def __init__(self):
        self._sub2 = rospy.Subscriber('chatter2', some_position , self.call)
        self._sub1 = rospy.Subscriber('chatter1', some_position2 ,self.callback)
        
        self._vel_pub = rospy.Publisher('diff_drive_controller/cmd_vel', Twist, queue_size=10)
        self._vel = Twist()
        self._red_area = 0
        self._blue_area = 0
        self.A = True
        
        #rate = rospy.Rate(50)  IMPORTANT roop velocity
        #rate.sleep()           last

    
    def callback(self, data):
        #rospy.loginfo(data.Class)                              #0

        Width=(data.xmax-data.xmin)
        Velocity=(((4.00-(((data.xmin + data.xmax)*0.01)/2))/2)*0.25*math.pi)
        
        start = time.time()
        
        if data.Class == "traffic light":
            #rospy.loginfo("test_stop!")
            #rospy.loginfo("11111111111!")                      #1

            #self._vel.angular.z = Velocity    ####----ADD----####
            #self._vel_pub.publish(self._vel)

            #if self._red_area > 250: #500: #700:
            #    rospy.loginfo("STOP!")                   #2
            #    self._vel.linear.x = 0
            #    self._vel_pub.publish(self._vel)
                #elif data.Class != "traffic light":
                #    self._vel.angular.z = 0.5
                #    self._vel_pub.publish(self._vel)    ####----ADD----####
                    #rospy.loginfo(self._x)
            elapsed_time = time.time() - start
            rospy.loginfo(' time : {} sec ' .format(elapsed_time))

            while self.A:
                elapsed_time = time.time() - start
                if self._red_area > 250 or self._blue_area < 150:
                    self._vel.linear.x = 0
                    self._vel_pub.publish(self._vel)
                    self.A = True
                    start2 = time.time()
                    elapsed_time2 = time.time() - start2
                    print(' time2 : {} sec ' .format(elapsed_time2))
                elif elapsed_time > 30 or elapsed_time > 60:
                    print(' time2 : {} sec  longer longer longer !!!' .format(elapsed_time2))
                    for i in range(1):
                        self._vel.angular.z = 0.8
                        self._vel_pub.publish(self._vel)

                if self._red_area  < 50:
                    if self._blue_area > 250:
                        self.A = False
                        break
                

                

            #rospy.sleep(2.0)
                        
                

    def call(self,msg1):
        #rospy.loginfo('red_area red_area red_area = %d' %(msg1.position[1]))  #3
        self._red_area = msg1.position[1]
        self._blue_area = msg1.position[0]
        #rospy.loginfo(self._red_area)                                         #4
    

def calculate(request):
        rospy.loginfo('called!')
        rospy.loginfo('Detect start...')
        position =[1,1,1]
        Class = 'Process Started'#'Process End!!'#'Detect Started!!'
        
        #pub = rospy.Publisher('chatter6', some_position, queue_size=10)
        #r = rospy.Rate(10) # 10hz
        
        #for num in range(101):
        #    msg = some_position()
        #    msg.header.stamp = rospy.Time.now()
        #    msg.position = [7,8,9]
        #    pub.publish(msg)
            #r.sleep()

        function = Integration()
        return KyoloResponse(position,Class)

if __name__ == '__main__':
    rospy.init_node('A_srv')

    service = rospy.Service('bbox', Kyolo , calculate)
    

    rospy.spin()