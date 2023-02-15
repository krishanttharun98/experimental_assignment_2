#!/usr/bin/env python3
"""
.. module:: helper.py
   :platform: ROS
   :synopsis: class for assistance features
   
.. moduleauthor::Krishant Tharun
 
This class offers a helper member that may be utilized to simplify the code of the program it is incorporated into.
The helper precisely provides all the actions clients need to control the robot in addition to other features for extracting information from the collected data and enquiries.
It's a tactic to prevent the overuse of global variables, which could complicate code reuse and potentially lead to bugs.
 
Subscribers:
    :state/battery_low- where the state of the battery is determined
Servers:
    :state/set_pose- server to set the current position of the robot in robot state node
  
"""







import rospy 
import time 
import random 
from armor_api.armor_client import ArmorClient
from Assignment import architecture_name_mapper as nm
from std_msgs.msg import Bool
from assignment.msg import PlanAction, ControlAction
from actionlib import SimpleActionClient
from threading import Lock
from assignment.srv import SetPose

client = ArmorClient("armor_client", "ontology")


class helperclient:
    """
    Class that simplifies the implementation of a client for ROS action servers.
    """

    def __init__(self, service_name, action_type, done_callback=None, feedback_callback=None, mutex=None):
        # Initialise the state of this client, i.e.,  `_running`, `_completed`, and `_results`
        self.client_reset()
        # Set the name of the server to be invoked
        self._service_name = service_name
        # Get or create a new mutex
        if mutex is None:
            self._mutex = Lock()
        else:
            self._mutex = mutex
        # Instantiate a simple ROS-based action client
        self._client = SimpleActionClient(service_name, action_type)
        # Set the done and feedback callbacks defined by the class using this client
        self._external_done_cb = done_callback
        self._external_feedback_cb = feedback_callback
        # Wait for the action server to be alive
        self._client.wait_for_server()

    def initialise_goal(self, goal):
        """
         A new goal can be given to the action server only if it is not running. This simplification implies that
          within the ROS architecture no more than one client can use the same server at the same time.
        Args:
            goal(PlanGoal): goal to be sent made up of two Points, start and target in (x, y) coordinates
        Returns:
            None
        """

        if not self._running:
            # Start the action server
            self._client.initialise_goal(goal,
                                   done_cb = self.feedback_complete,
                                   feedback_cb = self.feedback)
            # Set the client's states
            self._running = True
            self._completed = False
            self._results = None
        else:
            print("Warning send a new goal, cancel the current request first!")

    def goalcancel(self):
        """
        Fucntion to Stop the computation of the action server.
        
        Args:
            None
            
        Returns:
            None
        """
        
        if self._running:
            # Stop the computation
            self._client.cancel_all_goals()
            # Reset the client's state
            self.client_reset()
        else:
            print("Warning cannot cancel a not running service!")

    def client_reset(self):
        """
        Function to reset the client state variables stored in this class.
        
        Args:
            None
            
        Returns:
            None
        """

        self._running = False
        self._completed = False
        self._results = None

    def feedback(self, feedback):
        """
        Function called when the action server has to send a feedback to the client.
        
        Args:
            feedback: feedback message to be sent to the client
            
        Returns:
            None
        """

        self._mutex.acquire()
        try:
            #Eventually, call the method provided by the node that uses this action client to manage a feedback
            if self._external_feedback_cb is not None:
                self._external_feedback_cb(feedback)
        finally:
            # Realise the mutex to unblock ROS-based thread waiting on the same mutex
            self._mutex.release()

    def feedback_complete(self, status, results):
        """
        Function called when the action server has finished its computation.
        
        Args:
            status: status of the action server
            results: results from the action server
            
        Returns:
            None
        """
        
        self._mutex.acquire()
        try:
            # Set the client's states
            self._running = False
            self._completed = True
            self._results = results
            # Call the method provided by the node that uses this action client to manage a result
            if self._external_done_cb is not None:
                self._external_done_cb(status, results)
        finally:
            self._mutex.release()

    def completed(self):
        """
        Function Get `True` if the action server finished is computation, or `False` otherwise.
        
        Args:
            None
            
        Returns:
            Bool: 'True' if the action server finished its computation, 'False' otherwise
        """
        
        return self._completed

    def running(self):
        """
        Function Get `True` if the action server is running, or `False` otherwise.
        
        Args:
            None
            
        Returns:
            Bool: `True` if the action server is running, `False` otherwise
        """

        return self._running

    def action_result(self):
        """
        Function that gets the result of the action server.
        
        Args:
            None
            
        Returns:
            Result of the action server, if any, 'None' otherwise
        """

        if self._completed:
            return self._results
        else:
            print("Error: cannot get result")
            return None

class helpersync:
    """
    A class to decouple the implementation of the Finite State Machine to the stimulus might that
    lead to state transitions. This class manages the synchronization with subscribers and action servers.
    """

    def __init__(self):
        # Create a shared mutex to synchronize action clients and subscribers
        self.mutex = Lock()
        # Set the initial state
        self.state_reset()
        # Define the callback associated with the battery low ROS subscribers
        rospy.Subscriber(nm.TOPIC_BATTERY_LOW, Bool, self.battery)
        # Define the clients for the the plan and control action servers
        self.planner_client = helperclient(nm.ACTION_PLANNER, PlanAction, mutex=self.mutex)
        self.controller_client = helperclient(nm.ACTION_CONTROLLER, ControlAction, mutex=self.mutex)

    def state_reset(self):
        """
        Function to reset the stimulus for the battery stored as state variable. 
        This function assumes that no states of the Finite State Machine run concurrently.
        
        Args: 
            None
        
        Returns:
            None
        """

        self._battery_low = False

    def battery(self, msg):
        """
        Function for the subscriber to get messages published from the `robot_state` node into the `/state/battery_low/` topic.
        
        Args:
            msg(Bool): status of the battery
            
        Returns:
            None
        """
 
        # Acquire the mutex to assure the synchronization with the other subscribers and action clients
        self.mutex.acquire()
        try:
            # Get the battery level and set the relative state variable encoded in this class
            self._battery_low = msg.data
        finally:
            # Release the mutex to eventually unblock the other subscribers or action servers that are waiting
            self.mutex.release()

    def low_battery(self):
        """
        Fucntion to get the state variable encoded in this class about the battery level.
        
        Args:
            None
        
        Returns:
            Bool: `True` if the battery is low, `False` otherwise
        """

        return self._battery_low

    @staticmethod
    def robot_position(point):
        """
        Function to update the current position of the robot stored in the 'robot_state' node.
        
        Args:
            point(Point): point representing the robot pose in (x, y) coordinates
            
        Returns:
            None
        """

        # Wait for the server to be initialised
        rospy.wait_for_service(nm.SERVER_SET_POSE)
        try:
            # Call the service and set the current robot position
            service = rospy.ServiceProxy(nm.SERVER_SET_POSE, SetPose)
            service(point)
            print("Setting initial robot position")
        except rospy.ServiceException as e:
            print("Cannot set current robot position")
