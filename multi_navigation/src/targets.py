#!/usr/bin/env python

import rospy
import json
import actionlib
import threading
from geometry_msgs.msg import Pose, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

class TurtleBotNavigator:
    def __init__(self, namespace, targets):
        self.namespace = namespace
        self.targets = targets
        
        # Create an action client for move_base action for this turtlebot
        self.client = actionlib.SimpleActionClient('/{}/move_base'.format(self.namespace), MoveBaseAction)
        
        # Wait for the action server to start
        rospy.loginfo("Waiting for {} move_base action server...".format(self.namespace))
        self.client.wait_for_server()
        rospy.loginfo("Connected to {} move_base action server".format(self.namespace))

    def navigate(self, task_indices):
        for index in task_indices:
            if index < len(self.targets):
                target = self.targets[index]
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = target
                
                rospy.loginfo("Sending goal to {}/move_base: {}".format(self.namespace, target))
                self.client.send_goal(goal)
                self.client.wait_for_result()
                result = self.client.get_result()
                rospy.loginfo("Result for {}: {}".format(self.namespace, result))

def create_pose(x, y):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.w = 1.0  # Assuming no orientation needed
    return pose

def task_callback(msg, navigators):
    task_data = json.loads(msg.data)
    threads = []
    for tb_namespace, task_indices in task_data.items():
        if tb_namespace in navigators:
            thread = threading.Thread(target=navigators[tb_namespace].navigate, args=(task_indices,))
            thread.start()
            threads.append(thread)
    
    for thread in threads:
        thread.join()

def main():
    rospy.init_node('targets', anonymous=True)
    
    # Define the target points
    target_points = [
        create_pose(-3.2, 3.8), create_pose(-2.6, 3.8), create_pose(-1.2, 3.8),
        create_pose(-0.5, 3.8), create_pose(0.7, 3.8), create_pose(1.3, 3.8),
        create_pose(1.3, 3.1), create_pose(0.7, 3.1), create_pose(-0.5, 3.1),
        create_pose(-1.2, 3.1), create_pose(-2.6, 3.1), create_pose(-3.2, 3.1),
        create_pose(-3.2, 1.6), create_pose(-2.6, 1.6), create_pose(-1.2, 1.6),
        create_pose(-0.5, 1.6), create_pose(0.7, 1.6), create_pose(1.3, 1.6),
        create_pose(1.3, 1), create_pose(0.7, 1),
        # New additional points
        create_pose(-2.0, -1.0), create_pose(2.0, 0.0), create_pose(-0.5, 1.0),
        create_pose(-0.5, -1.0), create_pose(-1.0, -1.0), create_pose(1.0, -1.0),
        create_pose(0.0, 1.5), create_pose(0.5, 0.0), create_pose(1.0, 0.5),
        create_pose(1.5, 0.0)
    ]
    
    # List of namespaces for each TurtleBot
    namespaces = ['tb3_0', 'tb3_1']
    
    # Create and store navigators for each TurtleBot
    navigators = {ns: TurtleBotNavigator(ns, target_points) for ns in namespaces}
    
    # Subscribe to the task list topic
    rospy.Subscriber('/task_allocation', String, task_callback, navigators)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
