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



class Callback(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state YOLO START...')
        rospy.sleep(5.0)
        rospy.loginfo('waiting service...')
    
        rospy.wait_for_service('bbox')
        try:
            
            service = rospy.ServiceProxy('bbox', Kyolo)
            

            response = service() #client -> server -> client
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        #sum = service(a,b)
        #print '->',sum.resultz

        print '->',response.Class
        #print '->',response.position     #1
        return 'done'

class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','failed','aborted'])
        self.counter = 0
        
    def execute(self, userdata):
        rospy.loginfo('Executing state DETECT START...')
        rospy.sleep(2.0)
        action_client = actionlib.SimpleActionClient( 'action', Practice2Action )
        action_client.wait_for_server() # Wait until the server is ready     
        # Set GoUntilBumperGaol's instance
        goal = Practice2Goal() 

        #goal.target_vel.linear.x = 1.0 #0.8
        goal.timeout_sec = 10
    
        action_client.send_goal( goal ) # Send data ( Publish to topic bumper_action/goal )
        action_client.wait_for_result() # wait for result
        
        result = action_client.get_result()
        if result.bumper_hit: 
            rospy.loginfo( 'bumper hit!' )
            return 'succeeded'
            
        else:
            rospy.loginfo( 'faild!' )
            #self.counter += 1
            if self.counter < 3:
                self.counter += 1
                return 'failed'
            else:
                return 'aborted'
                #return 'failed'

        

#class State2(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['done'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state STATE2')
#        rospy.sleep(2.0)
#        return 'done'

def main():
    rospy.init_node('A_test_state_machine')
   
    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['done', 'exit'])
    
    # Open the container
    with sm_top:
        goal1=MoveBaseGoal()
        goal1.target_pose.header.frame_id = "map"
        goal1.target_pose.pose.position.x = -0.5777460336685181
        goal1.target_pose.pose.position.y = 5.013327121734619
        goal1.target_pose.pose.orientation.z = 0.7086065566521872
        goal1.target_pose.pose.orientation.w = 0.7056038179244289
        smach.StateMachine.add('Crossroad', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal1), transitions={'succeeded':'YOLO_START', 'preempted':'exit', 'aborted':'exit'})

        
        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:


            smach.StateMachine.add('YOLO', Callback(), transitions={'done':'done'})
        smach.StateMachine.add('YOLO_START', sm_sub, transitions={'done':'MOVE2'}) 

        goal2=MoveBaseGoal()
        goal2.target_pose.header.frame_id = "map"
        goal2.target_pose.pose.position.x = -0.6516845226287842
        goal2.target_pose.pose.position.y = 16.40520477294922
        goal2.target_pose.pose.orientation.z = 0.9012824269013652
        goal2.target_pose.pose.orientation.w = 0.43323202439199415
        smach.StateMachine.add('MOVE2', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal2), transitions={'succeeded':'MOVE3', 'preempted':'exit', 'aborted':'exit'})

        goal3=MoveBaseGoal()
        goal3.target_pose.header.frame_id = "map"
        goal3.target_pose.pose.position.x = -6.68888521194458
        goal3.target_pose.pose.position.y = 16.2919921875
        goal3.target_pose.pose.orientation.z = 0.9388371582918714
        goal3.target_pose.pose.orientation.w = -0.34436142381289414
        smach.StateMachine.add('MOVE3', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal3), transitions={'succeeded':'Serach area arrival', 'preempted':'exit', 'aborted':'exit'})

        








        goal4=MoveBaseGoal()
        goal4.target_pose.header.frame_id = "map"
        goal4.target_pose.pose.position.x = -10.202484130859375
        goal4.target_pose.pose.position.y = 14.14236831665039
        goal4.target_pose.pose.orientation.z = -0.8497615112390302
        goal4.target_pose.pose.orientation.w = 0.5271673112179465
        smach.StateMachine.add('Serach area arrival', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal4), transitions={'succeeded':'DETECT_AREA', 'preempted':'exit', 'aborted':'exit'})
         
        sm_sub = smach.StateMachine(outcomes=['done','exit'])
        with sm_sub:


            smach.StateMachine.add('Detect_Start', State1(), transitions={'succeeded':'done', 'failed':'exit', 'aborted':'done'})
            #smach.StateMachine.add('Retry', State2(), transitions={'done':'Detect_Start'})
        smach.StateMachine.add('DETECT_AREA', sm_sub, transitions={'done':'MOVE5','exit':'DETECT_AREA'}) 

        goal5=MoveBaseGoal()
        goal5.target_pose.header.frame_id = "map"
        goal5.target_pose.pose.position.x = -11.046177864074707
        goal5.target_pose.pose.position.y = 11.073451042175293
        goal5.target_pose.pose.orientation.z = 0.0410986462735627
        goal5.target_pose.pose.orientation.w = 0.9991550937039158
        smach.StateMachine.add('MOVE5', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal5), transitions={'succeeded':'MOVE6', 'preempted':'exit', 'aborted':'exit'})

        goal6=MoveBaseGoal()
        goal6.target_pose.header.frame_id = "map"
        goal6.target_pose.pose.position.x = -7.305649280548096
        goal6.target_pose.pose.position.y = 11.042054176330566
        goal6.target_pose.pose.orientation.z = -0.7081168395818828
        goal6.target_pose.pose.orientation.w = 0.706095277919748
        smach.StateMachine.add('MOVE6', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal6), transitions={'succeeded':'Crossroad2', 'preempted':'exit', 'aborted':'exit'})

        goal7=MoveBaseGoal()
        goal7.target_pose.header.frame_id = "map"
        goal7.target_pose.pose.position.x = -6.966019630432129
        goal7.target_pose.pose.position.y = 5.273207664489746
        goal7.target_pose.pose.orientation.z = -0.7419502719629223
        goal7.target_pose.pose.orientation.w = 0.6704549156611097
        smach.StateMachine.add('Crossroad2', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal7), transitions={'succeeded':'MOVE8', 'preempted':'exit', 'aborted':'exit'})

        goal8=MoveBaseGoal()
        goal8.target_pose.header.frame_id = "map"
        goal8.target_pose.pose.position.x = -6.711871147155762
        goal8.target_pose.pose.position.y = -0.7483253479003906
        goal8.target_pose.pose.orientation.z = -0.7699807246647065
        goal8.target_pose.pose.orientation.w = 0.6380671466584168
        smach.StateMachine.add('MOVE8', smach_ros.SimpleActionState('/move_base', MoveBaseAction, goal=goal8), transitions={'succeeded':'exit', 'preempted':'exit', 'aborted':'exit'})

# Execute SMACH plan
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
