#!/usr/bin/env python
import rospy
import random
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Mover:
    """
    Move action class. Responsible for autonomous navigation
    of robot and reaction to signs. Subscribes to the
    /sticky_wickets/sign/move_goal topic for sign reaction.
    Simple autonomy is accomplished using relative goal-based
    exploration using costmap and map.
    """
    def __init__(self, move_amount=0.4):
        self.goal_running = False
        self.sign_goal = None
        self.sign_detected = False

        self.current_action = "forward"
        self.action_wait_time = 3.5
        self.goal_move_amt = move_amount

        self.sub_goal = rospy.Subscriber("/sticky_wickets/sign/move_goal", MoveBaseGoal, self.sign_callback)
    	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for action server...")
    	self.move_base.wait_for_server(rospy.Duration(5))
        rospy.on_shutdown(self.shutdown)

    def sign_callback(self, goal):
        """
        Callback for sign detection.
        """
        if not self.goal_running:
            self.sign_detected = True
            self.sign_goal = goal

    def explore(self):
        """
        Entry point for autonomous movement.
        Responsible for constantly updating move goals.
        """
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.move_action(self.action_wait_time)

    def move_action(self, goal_cutoff_time):
        """
        Configures and runs a goal for the robot. If a sign
        must be reacted to, the goal is retrieved from the
        publisher (sign_handler), and overrides the current
        autonomous exploration goals.
        """
        rospy.loginfo("Initializing and sending goal...")
        goal = MoveBaseGoal()

        turn_amt = 0.75 * random.choice([-1, 1])

        if self.sign_detected:
            rospy.loginfo("Sign recieved: reacting...")
            print(self.sign_goal)
            goal = self.sign_goal
            self.sign_detected = False
        else:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'base_link'
            goal.target_pose.header.stamp = rospy.Time.now()

            if self.current_action == "forward":
                goal.target_pose.pose.position.x = self.goal_move_amt
                goal.target_pose.pose.orientation.z = 0.0
                goal.target_pose.pose.orientation.w = 1.0
            elif self.current_action == "rotate":
                goal.target_pose.pose.position.x = 0.0
                goal.target_pose.pose.orientation.z = turn_amt
                goal.target_pose.pose.orientation.w = 1.0

        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(goal_cutoff_time))

        if success and self.move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal Reached")
            self.current_action = "forward"
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("Goal Cancelled: will turn")
            self.current_action = "rotate"

    def shutdown(self):
        """
        Cancels current goals.
        """
        self.move_base.cancel_goal()
        rospy.loginfo("Stopping goal")
        rospy.sleep(1)


def main(args):
    try:
        rospy.init_node('mover', anonymous=False)
        navigator = Mover()
        navigator.explore()

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")


if __name__ == '__main__':
    main(sys.argv)
