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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import math

### delete unnecessary library


### global variables
route_path = '/root/catkin_ws/src/ros_beginner/src/route/'
route_num = 1


### make directory
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



### make goal file
class make_goal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit', 'next'])
        
        self.goal_pose = []

        self.marker_publisher = rospy.Publisher('set_marker', MarkerArray, queue_size=10)

    def execute(self, userdata):

        global route_path, route_num


        print("Please set goal by 2D Nav goal tool...")
        goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback_get_goal)

        while not self.goal_pose:
            rospy.sleep(0.5)
        print(self.goal_pose)


        txt_name = "/goal" + str(route_num)+ ".txt"
        goal_pose_str = [str(i) for i in self.goal_pose]
        
        with open(route_path + txt_name, 'a') as test_write:
            test_write.write(goal_pose_str[0] + ' ' + goal_pose_str[1] + ' ' + goal_pose_str[2] + ' ' + goal_pose_str[3] + ' ' + goal_pose_str[4] + ' ' + goal_pose_str[5] + ' ' + goal_pose_str[6])
       
        print("Succeeded in setting goal!!")
        del self.goal_pose[:]

        msg = """
        Enter -> Set target
          g   -> Set next goal
          q   -> Quit making route
        """
        print(msg)
        key = raw_input("key: ")

        if key == 'q':
            return 'exit'
        
        elif key == 'g':
            route_num += 1
            return 'next'
        
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
        marker.id = 1
        markerArray.markers.append(marker)
        self.marker_publisher.publish(markerArray)



### make target file
class make_target(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])

        self.flag_target_route = True

        self.target_pose = []

        self.marker_publisher = rospy.Publisher('set_marker', MarkerArray, queue_size=10)
        self.marker_id_target = 1

    def execute(self, userdata):

        global route_path, route_num
        flag_target_route = True
        return_done = False
        return_exit = False


        while flag_target_route:
            print("Please set target by 2D Pose Estimate tool...")

            target_subscriber = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.callback_get_target)

            while not self.target_pose:
                rospy.sleep(0.5)
            print(self.target_pose)

            txt_name = "/target" + str(route_num)+ ".txt"
            goal_pose_str = [str(i) for i in self.target_pose]

            with open(route_path + txt_name, 'a') as test_write:
                test_write.write(goal_pose_str[0] + ' ' + goal_pose_str[1] + ' ' + goal_pose_str[2] + ' ' + goal_pose_str[3] + ' ' + goal_pose_str[4] + ' ' + goal_pose_str[5] + ' ' + goal_pose_str[6] + '\n')

            print("Succeeded in setting target!!")
            del self.target_pose[:]

            msg = """
            Enter -> Set next target
              g   -> Set next goal
              q   -> Quit making route
            """
            print(msg)
            key = raw_input("key: ")

            if key == 'q':
                flag_target_route = False
                return_exit = True

            elif key == 'g':
                flag_target_route = False
                return_done = True

            else:
                pass

        if return_done:
            self.marker_id_target = 1
            route_num += 1
            return 'done'
        
        elif return_exit:
            self.marker_id_target = 1
            return 'exit'


    def callback_get_target(self, msg):

        print("Get target")
        target_pose_x = msg.pose.pose.position.x
        target_pose_y = msg.pose.pose.position.y
        target_pose_z = msg.pose.pose.position.z
        target_posi_x = msg.pose.pose.orientation.x
        target_posi_y = msg.pose.pose.orientation.y
        target_posi_z = msg.pose.pose.orientation.z
        target_posi_w = msg.pose.pose.orientation.w

        self.target_pose.extend([target_pose_x, target_pose_y, target_pose_z, target_posi_x, target_posi_y, target_posi_z, target_posi_w])

        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x    = self.target_pose[0]
        marker.pose.position.y    = self.target_pose[1]
        marker.pose.position.z    = self.target_pose[2]
        marker.pose.orientation.x = self.target_pose[3]
        marker.pose.orientation.y = self.target_pose[4]
        marker.pose.orientation.z = self.target_pose[5]
        marker.pose.orientation.w = self.target_pose[6]
        marker.id = self.marker_id_target + 1000
        markerArray.markers.append(marker)
        self.marker_publisher.publish(markerArray)

        self.marker_id_target += 1



# main
def main():
    rospy.init_node('make_route')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('make_dir', make_dir(), transitions={'done':'make_goal'})
        smach.StateMachine.add('make_goal', make_goal(), transitions={'done':'make_target', 'exit':'succeeded', 'next':'make_goal'})
        smach.StateMachine.add('make_target', make_target(), transitions={'done':'make_goal', 'exit':'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/navigation')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()