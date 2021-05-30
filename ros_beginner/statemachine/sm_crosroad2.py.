#!/usr/bin/env python

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Empty

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String

from geometry_msgs.msg import Twist

import math
import time

from ros_beginner.srv import Kyolo
import actionlib
from ros_beginner.msg import Practice2Action
from ros_beginner.msg import Practice2Goal
import csv
import tf
import pandas as pd


#a = 0
#class set_route(smach.State):

#    def __init__(self):
#        smach.State.__init__(self, outcomes=['done'])


#    def execute(self, userdata):

#        global route_name
#        global route_name2

#        print("Please input Crossroad Place...")
#        route_name = str(input("Crossroad: "))
#        route_name2 = str(input("Crossroad2: "))

#        return 'done'

class target_navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

        self.listener = tf.TransformListener()
        self.A = True
        self.B = True


        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) 
        self.client.wait_for_server()
        self.listener.waitForTransform("map", "/base_link", rospy.Time(), rospy.Duration(4.0))

        self.num = True
        self.num2 = True
        
        
        self.counter = 0


        with open('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', 'r') as f:
            #test_route = f.read().split()
            #self.test_route_array = [float(i) for i in test_route]
            #print(self.test_route_array)
            
            #f = open('dq-list.csv', 'r') 
            alltxt = f.readlines() 
            f.close() 
            endgyou = len(alltxt) 
            self.endtxt = alltxt[endgyou-1].strip() 
            #print(endtxt) 
            rospy.loginfo('LAST POINT : {}' .format(self.endtxt[0]))
            print('LAST POINT : {}' .format(self.endtxt[0]))

        print("Please input Crossroad Place...")
        self.Crossroad = str(input("Crossroad: ")) #'2'
        #print(int(self.endtxt[0])-2)
        #print(int(self.endtxt[0]))
        #if int(self.Crossroad) == int(self.endtxt[0])-1:

        while True:
            #int(self.Crossroad) >= int(self.endtxt[0])-1:
            #print('error!?')
            if int(self.Crossroad) >= int(self.endtxt[0])-1:
                print('error!?')
                self.Crossroad = str(input("Crossroad: ")) #'2'
            else:
                break



        self.Crossroad2 = str(input("Crossroad2: ")) #'3'

        #if int(self.Crossroad2) == int(self.endtxt[0]):
        while int(self.Crossroad2) >= int(self.endtxt[0]):
            print('error!')
            self.Crossroad2 = str(input("Crossroad2: ")) #'3'
            #print('error!')
            #break


        #with open('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', 'r') as f:

        self.df = pd.read_csv('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', skiprows=int(self.Crossroad))
        #f.close() 
            
        print(self.df)




            


        

    def execute(self, userdata):

        while self.A:
            with open('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', 'r') as f:
                reader = csv.reader(f)
         
                for pose in reader:

                    if not self.num:
                        #if self.num2:
                        if pose[0] <= self.Crossroad:
                            print('skip!!!!!')
                            continue

                    if not self.num2:
                        #if self.num2:
                        if pose[0] <= self.Crossroad2:
                            print('skip2222!!!!!')
                            continue


                    print("\nHeading to {}!".format(pose[0]))
                    goal = self.goal_pose(pose)
                    self.client.send_goal(goal)
                    while True:
                        now = rospy.Time.now()
                        self.listener.waitForTransform("map", "/base_link", now, rospy.Duration(4.0))
                        position, quaternion = self.listener.lookupTransform("map", "/base_link", now)
                        if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                            
                            #self.A = False

                            print("->Reached {}! Next head to {}!".format(pose[0],int(pose[0])+1))

                            rospy.loginfo(pose[0])

                            break
                        else:
                            rospy.sleep(0.5)

                    if pose[0] == self.Crossroad:    #Crossroad  self.crossroad == '2'
                        rospy.loginfo('AAAAAAAAAAAAAAAAAAAA') 
           
                        print(pose)
                        if self.num:
                         self.num = False
                         return 'done'

                    if pose[0] == self.Crossroad2:    #Crossroad  self.crossroad == '2'
                        rospy.loginfo('BBBB!!!!') 
           
                        print(pose)
                        if self.num2:
                         self.num2 = False
                         return 'done'
                         

                    if pose[0] == self.endtxt[0]: #'4':
                        
                        self.A = False
                            
                rospy.loginfo("Finish")

        return 'exit'
        

    def goal_pose(self, pose):

        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = map(float,pose)[1]
        goal_pose.target_pose.pose.position.y = map(float,pose)[2]
        goal_pose.target_pose.pose.position.z = map(float,pose)[3]
        goal_pose.target_pose.pose.orientation.x = map(float,pose)[4]
        goal_pose.target_pose.pose.orientation.y = map(float,pose)[5]
        goal_pose.target_pose.pose.orientation.z = map(float,pose)[6]
        goal_pose.target_pose.pose.orientation.w = map(float,pose)[7]

        return goal_pose
        
        





# main
def main():
    rospy.init_node('navigation')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        #smach.StateMachine.add('Crossroad', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'YOLO_START', 'preempted':'exit', 'aborted':'exit'})
        #smach.StateMachine.add('MOVE5', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal5), transitions={'succeeded':'MOVE6', 'preempted':'exit', 'aborted':'exit'})
        #smach.StateMachine.add('set_route', set_route(), transitions={'done':'target_navigation'})

        #smach.StateMachine.add('set_route', set_route(), transitions={'done':'target_navigation'})
        smach.StateMachine.add('target_navigation', target_navigation(), transitions={'done':'target_navigation', 'exit':'succeeded'})
        #smach.StateMachine.add('goal_navigation', goal_navigation(), transitions={'next':'target_navigation', 'done':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
