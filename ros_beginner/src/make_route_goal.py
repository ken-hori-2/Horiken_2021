#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import Int8
import sys, select, os
import tty, termios
from visualization_msgs.msg import Marker, MarkerArray
import tf
import tf2_ros
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PointStamped, PoseWithCovarianceStamped, TransformStamped, PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from nav_msgs.msg import Odometry
import math

### delete unnecessary library


route_path = '/root/catkin_ws/src/ros_beginner/src/route1/'


class make_dir(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])


    def execute(self, userdata):
        print("Please input place name...")
        place_name = raw_input("place name: ")

        global route_path
        route_path = route_path + place_name ### add place name to global route path
        os.mkdir(route_path)

        print("Succeeded in making directory!!")
        return 'done'


class make_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])
        
        self.goal_pose = []
        self.goal_count = 1

    def execute(self, userdata):
        print("Please set goal by 2D Nav goal tool...")

        goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback_get_goal)

        while not self.goal_pose:
            rospy.sleep(0.5)
        print(self.goal_pose)

        txt_name = "/goal" + str(self.goal_count)+ ".txt"
        goal_pose_str = [str(i) for i in self.goal_pose]
        
        global route_path
        with open(route_path + txt_name, 'a') as test_write:
            test_write.write(goal_pose_str[0] + ' ' + goal_pose_str[1] + ' ' + goal_pose_str[2] + ' ' + goal_pose_str[3] + ' ' + goal_pose_str[4] + ' ' + goal_pose_str[5] + ' ' + goal_pose_str[6])
       
        print("Succeeded in setting goal!!")
        del self.goal_pose[:]
        self.goal_count += 1

        msg = """
        Enter -> Set goal
          t   -> Set target
          q   -> Quit making route
        """
        print(msg)
        key = raw_input("key: ")

        if key == 'q':
            return 'exit'
        
        else:
            return 'done'



    def callback_get_goal(self, msg):

        print("Get goal")
        goal_pose_x = msg.pose.position.x
        goal_pose_y = msg.pose.position.y
        goal_pose_z = msg.pose.position.z
        goal_posi_x = msg.pose.orientation.x
        goal_posi_y = msg.pose.orientation.y
        goal_posi_z = msg.pose.orientation.z
        goal_posi_w = msg.pose.orientation.w

        self.goal_pose.extend([goal_pose_x, goal_pose_y, goal_pose_z, goal_posi_x, goal_posi_y, goal_posi_z, goal_posi_w])

        marker_publisher = rospy.Publisher('set_marker', MarkerArray, queue_size=10)

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x    = self.goal_pose[0]
        marker.pose.position.y    = self.goal_pose[1]
        marker.pose.position.z    = self.goal_pose[2]
        marker.pose.orientation.x = self.goal_pose[3]
        marker.pose.orientation.y = self.goal_pose[4]
        marker.pose.orientation.z = self.goal_pose[5]
        marker.pose.orientation.w = self.goal_pose[6]
        marker.id = self.goal_count
        markerArray.markers.append(marker)
        marker_publisher.publish(markerArray)



# main
def main():
    rospy.init_node('make_route')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('make_dir', make_dir(), transitions={'done':'make_goal'})
        smach.StateMachine.add('make_goal', make_goal(), transitions={'done':'make_goal', 'exit':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
