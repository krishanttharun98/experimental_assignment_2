#!/usr/bin/env python3
"""
.. module:: fsm.py
   :platform: Unix
   :synopsis: code for initialising the finite state machine of the robot 
.. moduleauthor:: Krishant Tharun
The finite state machine's structure is specified using this script. Here, all of its states and associated transitions are initialized.
To prevent the system from being stuck or experiencing faults during changes, it is defined how to behave for each state during each transition that is received.
In order to facilitate the use of functions and shared variables in particular situations, a helper entity is also created at the start of the execution and supplied as a parameter to each state. An attribute of the relevant Class:mod:'helper' is this entity.
The primary function of the helper, however, is to share the mutex that is required to access shared variables without difficulty. Of fact, there is only one mutex employed in this attempt to achieve complete synchronization between the state and the reading and writing operations.
 
"""

import smach
import rospy
import random
import smach_ros
import time
from smach import StateMachine,State
from helper import InterfaceHelper
from query_helper import OntologyHelper
from ontology_build import build_ontology_map
from assignment.msg import Point, ControlGoal, PlanGoal
from armor_api.armor_client import ArmorClient
from Assignment import architecture_name_mapper as nm

#list of possible states in the machine
STATE_INITIALIZE = 'waiting_for_map'
STATE_CHOOSE_CORRIDOR = 'corridor_decider'
STATE_CHOOSE_URGENT = 'room_decider'
STATE_MOVING_CORRIDOR = "moving_to_corridor"
STATE_MOVING_URGENT = "moving_to_room"
#STATE_CORRIDOR = 'CORRIDOR'
STATE_RECHARGING = 'charging'
START_URGENT='urgent_visit'
#STATE_ROOM='ROOM'
# list of transition states
TRANS_INITIALIZED = 'map_ready'
TRANS_DECIDED_CORRIDOR = 'decided_corridor'
#TRANS_MOVED_CORRIDOR = 'robot_moved_corridor'
#TRANS_MOVED_URGENT= 'robot_moved_urgent'
TRANS_DECIDED_URGENT = 'urgent_visit'
TRANS_BATTERY_LOW = 'battery_low'
TRANS_RECHARGING = 'charging'
TRANS_RECHARGED = 'battery_full'

# Sleeping time (in seconds) of the waiting thread to allow the computations
# for getting stimulus from the other components of the architecture.
LOOP_SLEEP_TIME = 0.3

class ontologybuild(smach.State):
    """
    A class to load the ontology map to the rbot
    """
    def __init__(self):

        
        State.__init__(self, outcomes = [TRANS_INITIALIZED])

    def execute(self, userdata):
        """
        Function responsible of the loading of the
        environment. It calls the build_ontology_map() which helps for the robot to know the environment
        Args:
            userdata: not used
        Returns:
            TRANS_INITIALIZED(str): transits to STATE_NORMAL which is a nested statemachine
        """
        build_ontology_map()
        print("ONTOLOGY MAP LOADED")
        return TRANS_INITIALIZED
    

class corridor_decision_maker(smach.State):
    """
    A class for the root to decide which corridor it should reach
    """

    def __init__(self, interface_helper, ontology_helper):
        
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_DECIDED_CORRIDOR,TRANS_DECIDED_URGENT], output_keys = ['robot_position', 'target','random_plan'])

    def execute(self, userdata):
        """
        Function responsible of the transitions between the 
        STATE_CHOOSE_CORRIDOR and the STATE_RECHARGING or STATE_MOVING_CORRIDOR or STATE_CHOOSE_URGENT.
        In this function theere are three possible transitions where if the battery is low,the state will be STATE_RECHARGING,if the target is
        a corridor the state will be STATE_MOVING_CORRIDOR and if the target is a urgent room he state will be STATE_CHOOSE_URGENT.A planner is used and it plans a path to be followed
        Args:
            userdata: used for output_keys to pass data to the other states.
        Returns:
            TRANS_RECHARGING (str): transition to STATE_RECHARGING.
        
        Returns:
            TRANS_DECIDED_URGENT (str): transition to the STATE_DECISION_URGENT.
            
        .Returns:
            TRANS_DECIDED_CORRIDOR (str): transition to the STATE_DECISION_CORRIDOR.
        """
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        robot_position, target = self._ontology.choose_target()
        userdata.robot_position = robot_position
        userdata.target = target
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                # If the target is a Rooom then planner cancels goals and transition TRANS_DECIDED_URGENT occurs
                if target=='R1' or target=='R2' or target=='R3' or target=='R4':
                        self._helper.planner_client.cancel_goals()
                        return TRANS_DECIDED_URGENT	
                 # If the planner finishes its computation, then take the TRANS_DECIDED_CORRIDOR transition
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED_CORRIDOR

            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


class movetocorridor(smach.State):
    """
    A class to move he robot to a cooridor which has been acquired
    """

    def __init__(self, interface_helper, ontology_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper

        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED_CORRIDOR], input_keys = [ "random_plan",'robot_position', 'target'], output_keys = ['robot_position'])

    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_MOVING_CORRIDOR and the STATE_RECHARGING or STATE_CHOOSE_CORRIDOR.
        this class is responsible for the robot to move to decided corridor and stays for specific time.A controller is used to enable the movement of the robot
        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            TRANS_RECHARGING(str): transition to the STATE_RECHARGING.
        Returns:
            TRANS_MOVED_CORRIDOR(str): transition to STATE_DECISION_CORRIDOR.
        """
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points = plan)
        self._helper.controller_client.send_goal(goal)
        robot_position = userdata.robot_position
        target = userdata.target

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    self._ontology.move_location(target, robot_position)
                    #the robot stays in a corridr for specific time period
                    rospy.sleep(anm.MONITOR_TIME)
                    return TRANS_MOVED_CORRIDOR
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)



class room_decision_maker(smach.State):
    """
    A class for the root to decide which Urgent room it should reach
    """


    def __init__(self, interface_helper, ontology_helper):
        
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper
        # Get the environment size from ROS parameters.
        self.environment_size = rospy.get_param('config/environment_size')
        State.__init__(self, 8outcomes = [TRANS_RECHARGING, TRANS_DECIDED_URGENT,TRANS_DECIDED_CORRIDOR], output_keys = ['robot_position', 'target','random_plan'])

    def execute(self, userdata):
        """
        Function responsible of the transitions between the 
        STATE_CHOOSE_URGENT and the STATE_RECHARGING or STATE_MOVING_URGENT or STATE_CHOOSE_CORRIDOR.
        In this function theere are three possible transitions where if the battery is low,the state will be STATE_RECHARGING,if the target is
        a corridor the state will be STATE_DECISON_CORRIDOR and if the target is a urgent room he state will be STATE_MOVING_URGENT.A planner is used and it plans a path to be followed
        Args:
            userdata: used for output_keys to pass data to the other states.
        Returns:
            TRANS_RECHARGING (str): transition to STATE_RECHARGING.
        
        Returns:
            TRANS_DECIDED_URGENT (str): transition to the STATE_DECISION_URGENT.
            
        .Returns:
            TRANS_DECIDED_CORRIDOR (str): transition to the STATE_DECISION_CORRIDOR.
        """
        # Define a random point to be reached through some via-points to be planned.
        goal = PlanGoal()
        goal.target = Point(x = random.uniform(0, self.environment_size[0]),
                            y = random.uniform(0, self.environment_size[1]))
        robot_position, target = self._ontology.choose_target()
        userdata.robot_position = robot_position
        userdata.target = target
        # Invoke the planner action server.
        self._helper.planner_client.send_goal(goal)
        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.planner_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                #if the target is a corridor then planner cancels goal and transition TRANS_DECIDED_CORRIDOR occurs
                if  target=='C1' or target=='C2' or target=='E':
                    self._helper.planner_client.cancel_goals()
                    return TRANS_DECIDED_CORRIDOR
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.planner_client.is_done():
                    userdata.random_plan = self._helper.planner_client.get_results().via_points
                    return TRANS_DECIDED_URGENT


            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)


class move_to_urgent_room(smach.State):
    """
    A class to move the robot to a Urgent room which has been acquired
    """

    def __init__(self, interface_helper, ontology_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper

        State.__init__(self, outcomes = [TRANS_RECHARGING, TRANS_MOVED_URGENT], input_keys = [ "random_plan",'robot_position', 'target'], output_keys = ['crobot_position'])

    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_MOVING_URGENT and the STATE_RECHARGING or STATE_CHOOSE_URGENT.
        this class is responsible for the robot to move to decided corridor and stays for specific time.A controller is used to enable the movement of the robot
        Args:
            userdata: for input_keys and output_keys to get data and pass data.
        
        Returns:
            TRANS_RECHARGING(str): transition to the STATE_RECHARGING.
        Returns:
            TRANS_MOVED_URGENT(str): transition to STATE_DECISION_URGENT.
        """
        # Get the plan to a random position computed by the `PLAN_TO_RANDOM_POSE` state.
        plan = userdata.random_plan
        # Start the action server for moving the robot through the planned via-points.
        goal = ControlGoal(via_points = plan)
        """
        ControlGoal: via points to reach the goal 
        """
        self._helper.controller_client.send_goal(goal)
        robot_position = userdata.robot_position
        target = userdata.target

        while not rospy.is_shutdown():
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is low, then cancel the control action server and take the `battery_low` transition.
                if self._helper.is_battery_low():  # Higher priority
                    self._helper.controller_client.cancel_goals()
                    self._ontology.go_to_recharge(robot_position)
                    return TRANS_RECHARGING
                # If the controller finishes its computation, then take the `went_random_pose` transition, which is related to the `repeat` transition.
                if self._helper.controller_client.is_done():
                    self._ontology.move_location(target, robot_position)
                    #the robot stays in  a urgent room for some time
                    rospy.sleep(anm.MONITOR_TIME)
                    return TRANS_MOVED_URGENT
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)




class charging(State):
    """
    A class for executing Recharging for the robot
    """

    def __init__(self, interface_helper, ontology_helper):
        # Get a reference to the interfaces with the other nodes of the architecture.
        self._helper = interface_helper
        self._ontology = ontology_helper
        # Initialise this state with possible transitions (i.e., valid outputs of the `execute` function).
        State.__init__(self, outcomes = [TRANS_RECHARGED])

    # Define the function performed each time a transition is such to enter in this state.
    # Note that the input parameter `userdata` is not used since no data is required from the other states.
    def execute(self, userdata):
        """
        Function responsible of the transition between the 
        STATE_RECHARGING to the STATE_NORMAL.
	the robot will be in this state until the battery is enough
 
        Args:
            userdata: not used
        Returns:
            TRANS_RECHARGED(str): transition to STATE_NORMAL
        """
        while not rospy.is_shutdown():  # Wait for stimulus from the other nodes of the architecture.
            # Acquire the mutex to assure data consistencies with the ROS subscription threads managed by `self._helper`.
            self._helper.mutex.acquire()
            try:
                # If the battery is no low anymore take the `charged` transition.
                if not self._helper.is_battery_low():
                    self._helper.reset_states()  # Reset the state variable related to the stimulus.
                    return TRANS_RECHARGED
            finally:
                # Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
                self._helper.mutex.release()
            # Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
            rospy.sleep(LOOP_SLEEP_TIME)

def main():
    """
    This function creates the state machine and defines all the transitions. Here two nested state machine are created.
    """
    rospy.init_node('Finite_state_machine', log_level = rospy.INFO)
    # Initialise an classes to manage the interfaces with the other nodes in the architecture.
    helper = InterfaceHelper()
    ontology = OntologyHelper()
    sm_main = StateMachine([])
    with sm_main:

        StateMachine.add(STATE_INITIALIZE, ontologybuild(),
                         transitions = {TRANS_INITIALIZED: STATE_CORRIDOR})
        
        sm_corridor = StateMachine(outcomes=[TRANS_BATTERY_LOW,START_URGENT])

        with sm_corridor:
            StateMachine.add(STATE_CHOOSE_CORRIDOR, corridor_decision_maker(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_DECIDED_CORRIDOR: STATE_MOVING_CORRIDOR,TRANS_DECIDED_URGENT:START_URGENT})
            StateMachine.add(STATE_MOVING_CORRIDOR, movetocorridor(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_MOVED_CORRIDOR: STATE_CHOOSE_CORRIDOR})
                                            
        sm_room = StateMachine(outcomes=[TRANS_BATTERY_LOW, START_CORRIDOR])
	
        with sm_room:
            StateMachine.add(STATE_CHOOSE_URGENT, room_decision_maker(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_DECIDED_URGENT: STATE_MOVING_URGENT,TRANS_DECIDED_CORRIDOR: START_CORRIDOR})
            StateMachine.add(STATE_MOVING_URGENT, move_to_urgent_room(helper, ontology),
                            transitions = {TRANS_RECHARGING : TRANS_BATTERY_LOW,
                                            TRANS_MOVED_URGENT: STATE_CHOOSE_URGENT})
           
        StateMachine.add(STATE_CORRIDOR, sm_corridor,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING,START_URGENT:STATE_ROOM})
        StateMachine.add(STATE_ROOM, sm_room,
                         transitions={TRANS_BATTERY_LOW: STATE_RECHARGING,START_CORRIDOR:STATE_CORRIDOR})
            
        StateMachine.add(STATE_RECHARGING, charging(helper, ontology),
                            transitions = {TRANS_RECHARGED: STATE_CORRIDOR})
    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('sm_introspection', sm_main, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm_main.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
   
if __name__ == "__main__":
    main()
