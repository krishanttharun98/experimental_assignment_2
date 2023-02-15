#!/usr/bin/env python3

"""
.. module::robot_states.py
   :platform: ROS
   :synopsis:: code to compute the path for the controller server
.. moduleauthor:: Krishant Tharun
Publisher:
    /state/battery_low to know the battery status
Servers:
    /state/set_pose: server to set the current robot position
    /state/get_pose: server to get the current robot position
"""

import threading
import random
import rospy
# Import constant name defined to structure the architecture.
from Assignment import architecture_name_mapper as nm
# Import the messages used by services and publishers.
from std_msgs.msg import Bool
from assignment.srv import GetPose, GetPoseResponse, SetPose, SetPoseResponse
from helper import helpersync
from assignment.msg import Point


# A tag for identifying logs producer.
LOG_TAG = nm.NODE_ROBOT_STATE


# The node manager class.
# This class defines two services to get and set the current 
# robot pose, and a publisher to notify that the battery is low.
class RobotState:

    def __init__(self):
        # Initialise this node.
        rospy.init_node(nm.NODE_ROBOT_STATE, log_level=rospy.INFO)
        # Initialise robot position.
        self._pose = None
        # Initialise battery level.
        self._battery_low = False
        # Initialise randomness, if enabled.
        self._randomness = rospy.get_param(nm.PARAM_RANDOM_ACTIVE, True)
        if self._randomness:
            self._random_battery_time = rospy.get_param(nm.PARAM_BATTERY_TIME, [15.0, 40.0])
        # Define services.
        rospy.Service(nm.SERVER_GET_POSE, GetPose, self.get_pose)
        rospy.Service(nm.SERVER_SET_POSE, SetPose, self.set_pose)
        # Start publisher on a separate thread.
        th = threading.Thread(target=self.is_battery_low)
        th.start()
        # Log information.
        log_msg = (f'Initialise node `{nm.NODE_ROBOT_STATE}` with services `{nm.SERVER_GET_POSE}` and '
                   f'`{nm.SERVER_SET_POSE}`, and topic {nm.TOPIC_BATTERY_LOW}.')
        rospy.loginfo(nm.tag_log(log_msg, LOG_TAG))

    # The `robot/set_pose` service implementation.
    # The `request` input parameter is the current robot pose to be set,
    # as given by the client. This server returns an empty `response`.
    def set_pose(self, request):
        if request.pose is not None:
            # Store the new current robot position.
            self._pose = request.pose
        else:
            rospy.logerr(nm.tag_log('Cannot set an unspecified robot position', LOG_TAG))
        # Return an empty response.
        return SetPoseResponse()

    # The `robot/get_pose` service implementation.
    # The `request` input parameter is given by the client as empty. Thus, it is not used.
    # The `response` returned to the client contains the current robot pose.
    def get_pose(self, request):
        # Log information.
        if self._pose is None:
            rospy.logerr(nm.tag_log('Cannot get an unspecified robot position', LOG_TAG))

        response = GetPoseResponse()
        response.pose = self._pose
        return response

    # Publish changes of battery levels. This method runs on a separate thread.
    def is_battery_low(self):
        # Define a `lathed` publisher to wait for initialisation and publish immediately.
        publisher = rospy.Publisher(nm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
        if self._randomness:
            # Publish battery level changes randomly.
            self.battery_notifier(publisher)

    # Publish when the battery change state (i.e., high/low) based on a random
    # delay within the interval [`self._random_battery_time[0]`, `self._random_battery_time[1]`).
    # The message is published through the `publisher` input parameter and is a
    # boolean value, i.e., `True`: battery low, `False`: battery high.
    def battery_notifier(self, publisher):
        delay = 0  # Initialised to 0 just for logging purposes.
        while not rospy.is_shutdown():
            # Publish battery level.
            publisher.publish(Bool(self._battery_low))
            # Log state.
            if self._battery_low:
                print("Robot got low battery after",delay,"seconds")
            else:
                print("Robot got a fully charged battery after",delay, "seconds")
            # Wait for simulate battery usage.
            if self._randomness:
            	delay = random.uniform(self._random_battery_time[0], self._random_battery_time[1])
            rospy.sleep(delay)
            # Change battery state.
            self._battery_low = not self._battery_low

    
if __name__ == "__main__":
    # Instantiate the node manager class and wait.
    RobotState()
    helper = helpersync()

    # Initialize robot position
    robot_pose_param = rospy.get_param(nm.PARAM_INITIAL_POSE, [0, 0])
    helper.robot_position(Point(x=robot_pose_param[0], y=robot_pose_param[1]))
    rospy.spin()

