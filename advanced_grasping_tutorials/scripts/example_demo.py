#!/usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pal_bt_grasping_msgs.msg import GraspObjectAction, GraspObjectGoal, GraspObjectResult


if __name__ == "__main__":

    rospy.init_node('example_demo_node')


    # Action clients definition
    grasp_action_name = "/advanced_grasping/example_grasp_action"

    grasp_client = actionlib.SimpleActionClient(
        grasp_action_name, GraspObjectAction)
    grasp_client.wait_for_server()

    # Setup goal and send to server
    grasp_goal = GraspObjectGoal()
    grasp_goal.object_id = "object"
    grasp_client.send_goal(grasp_goal)

    rospy.loginfo("Sending grasp goal")
    grasp_client.wait_for_result()

    grasp_result = GraspObjectResult()

    if not grasp_client.get_state() == GoalStatus.SUCCEEDED:
        grasp_result = grasp_client.get_result()
        rospy.logerror(grasp_result.error_msg)
    else:
        rospy.loginfo("Object grasped successfully")

    rospy.logerr("Example demo finished")
    