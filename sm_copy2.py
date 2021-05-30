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

class target_navigation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

        self.listener = tf.TransformListener()
        self.A = True
        self.B = True


        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction) 
        self.client.wait_for_server()
        self.listener.waitForTransform("map", "/base_link", rospy.Time(), rospy.Duration(4.0))

        self.num = []
        self.a = '2'
        
        self.counter = 0

        

    def execute(self, userdata):

        while self.A:
            with open('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', 'r') as f:
                reader = csv.reader(f)
                for pose in reader:
                    print("\nHeading to {}!".format(pose[0]))
                    goal = self.goal_pose(pose)
                    self.client.send_goal(goal)
                    while True:
                        now = rospy.Time.now()
                        self.listener.waitForTransform("map", "/base_link", now, rospy.Duration(4.0))
                        position, quaternion = self.listener.lookupTransform("map", "/base_link", now)
                        if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                            
                            self.A = False

                            print("->Reached {}! Next head to {}!".format(pose[0],int(pose[0])+1))

                            rospy.loginfo(pose[0])

                            break
                        else:
                            rospy.sleep(0.5)

                    #if self.counter == 0:
                        #print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                        
                        

                    if pose[0] == self.a:    #Crossroad  self.a == '2'
                        rospy.loginfo('AAAAAAAAAAAAAAAAAAAA') 
                        
                        #break
                        #self.counter += 1
                        print(pose)
                        #if self.num:
                        # self.num = False
                        return 'done'
                            
                rospy.loginfo("Finish")

        while self.B:
                
            with open('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', 'r') as f:
                reader = csv.reader(f)
                #read = pd.read_csv('/root/catkin_ws/src/ros_beginner/src/wp/WP2.csv', skiprows=2)
                for pose in reader:

                    #if pose[0] == '0' or pose[0] == '1' or pose[0] == '2':
                    if pose[0] <= self.a:
                        print('skip!!!!!')
                        continue
                    print(pose)
                    print("\nHeading to {}!".format(pose[0]))
                    goal = self.goal_pose(pose)
                    self.client.send_goal(goal)
                    while True:
                        now = rospy.Time.now()
                        self.listener.waitForTransform("map", "/base_link", now, rospy.Duration(4.0))
                        position, quaternion = self.listener.lookupTransform("map", "/base_link", now)
                        if(math.sqrt((position[0]-goal.target_pose.pose.position.x)**2 + (position[1]-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                            
                            self.B = False

                            print("->Reached {}! Next head to {}!".format(pose[0],int(pose[0])+1))

                            rospy.loginfo(pose[0])

                            break
                        else:
                            rospy.sleep(0.5)


        #return 'done'
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
        smach.StateMachine.add('target_navigation', target_navigation(), transitions={'done':'target_navigation', 'exit':'succeeded'})
        #smach.StateMachine.add('goal_navigation', goal_navigation(), transitions={'next':'target_navigation', 'done':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
