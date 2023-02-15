import random
import time
import re
import sys
import rospy
import roslib
from armor_api.armor_client import ArmorClient
from Assignment import architecture_name_mapper as anm
from assignment.msg import Point
client = ArmorClient("armor_client", "ontology")


class OntologyHelper:
    """
    A class to used to query an use the armor client commands
    """

    def __init__(self):
        
        self.robot = "Robot1"
        self.charging_room = "E"
        self.urgent = 'urgent'
        self.corridor = 'corridor'
        self.robot_position = 'robot_position'
        self.reachable_locations = 'reachable_locations'

 
        
    def list_formatter(self, oldlocation, start,end):
        """
        This formatter strores the return value when the armor is queried because the return value be of many unwanted symbols
        Args:
            newlocation(str): list which will hold he location and everytime the armor is queried it stores the location alone
        Returns:
            new newlocation[] everytime the system is quried
        """
        newlocation = []
        for location in oldlocation:
        	newlocation.append(re.search(start + '(.+?)' + end,location).group(1))
        return newlocation
            
    
    


	
    def location_acquire(self, location):
        """
	the locations are queried to the armor 
        Args:
            location(str): a string to know whether the location is subjected to urgentrooms or corridors.
        Returns:
            location_list(str): list of queried locations.
        """

        if location == self.corridor:
            corridors_list = self.list_formatter(client.query.ind_b2_class('CORRIDOR'), '#', '>')
            location_list =  corridors_list
            
        if location == self.urgent:
            urgent_rooms = self.list_formatter(client.query.ind_b2_class('URGENT'), '#', '>')
            location_list = urgent_rooms

        if location == self.robot_position:
            current_position = self.list_formatter(client.query.objectprop_b2_ind('isIn',self.robot), '#', '>')[0]
            location = current_position
            location_list = current_position
            
        if location == self.reachable_locations:
            reachable_destinations =  self.list_formatter(client.query.objectprop_b2_ind('canReach',self.robot), '#', '>')
            location_list = reachable_destinations

        return location_list


    def choose_target(self):
        """
        The function that helps to predict the robot;s movement
        Returns:
            current_position(str): The current rposition of robot
        Returns:
            target(str): The target position chosen from a list of reachable and urgent locations
        
        Returns:
            corridors(str): all the corridors
        """

        corridors_list = self.location_acquire(self.corridor)
        current_position = self.location_acquire(self.robot_position)
        print("\ncurrent position of robot is: ["+(current_position)+"]")
        if current_position=='E':
        	print("\ncoordinate is: (0,0)")
        if current_position=='C1':
        	print("\ncoordinate is: (1,1)")
        if current_position=='C2':
        	print("\ncoordinate is:(-1,1)")
        if current_position=='R1':
        	print("\ncoordinate is:(2,1)")
        if current_position=='R2':
        	print("\ncoordinate is:(2,2)")
        if current_position=='R3':
        	print("\ncoordinate is:(-2,1)")
        if current_position=='R4':
        	print("\ncoordinate is:(-2,2)")
        possible_destinations = self.location_acquire(self.reachable_locations)
        print("Reachable Positions: [" + ", ".join(possible_destinations) +"]")
        urgent_rooms = self.location_acquire(self.urgent)
        print("URGENCY ROOMS: [" + ", " .join(urgent_rooms) + "]")
        ReachableUrgent_room=[i for i in possible_destinations if i in urgent_rooms]
        if not ReachableUrgent_room:
            # if the list of possible destination 
            # contains only one element
            ReachableCorridor_room = [i for i in possible_destinations if i in corridors_list]
            if not ReachableCorridor_room:
            	target = random.choice(possible_destinations)
            else:
                target = random.choice(ReachableCorridor_room)
        # if chosen_target list is not empty
        else:
            # save the first element of the list as the oldest timestamp
            oldest = client.query.dataprop_b2_ind('visitedAt', ReachableUrgent_room[0])
            # clean the string
            oldest =  str(self.list_formatter(oldest, '"', '"')[0])
            for i in range (len(ReachableUrgent_room)):
                choice_last_visit = client.query.dataprop_b2_ind('visitedAt', ReachableUrgent_room[i])
                choice_last_visit =  str(self.list_formatter(choice_last_visit, '"', '"')[0])
                if choice_last_visit <= oldest:
                    target = ReachableUrgent_room[i]
        print("Target to be Reached " + target)
        return current_position, target


    def move_location(self, target, current_position):
        """
        Function used to make the robot move to the acquired target
        Args:
            target(str): The target position chosen from a reachable and urgent locations
            current_pos(str): The current robot position obtained from the ontology
            corridors(str): All corridors  in the map
        """
        

       
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, target, current_position)
        print("Robot in " + target + " and monitoring")
        last_change = client.query.dataprop_b2_ind('now', self.robot)
        last_change =  str(self.list_formatter(last_change, '"', '"')[0])
        current_time = str(int(time.time()))
        client.manipulation.replace_dataprop_b2_ind('now', self.robot, 'Long', current_time, last_change)
        corridors=self.list_formatter(client.query.ind_b2_class('CORRIDOR'), '#', '>')
        if target not in corridors:
            last_visit = client.query.dataprop_b2_ind('visitedAt', target)
            last_visit =  str(self.list_formatter(last_visit, '"', '"')[0])
            client.manipulation.replace_dataprop_b2_ind('visitedAt',target,'Long',current_time,last_visit)
        client.utils.apply_buffered_changes()
        client.utils.sync_buffered_reasoner()

    def go_to_recharge(self, robot_location):
        """
        Function to move the robot toits charging room location
        Args:
            robot_location(str): The current robot position obtained f
        """
        print("\nRobot's Battery is low and going to recharge room for recharge ")
        client.manipulation.replace_objectprop_b2_ind('isIn', self.robot, self.charging_room, robot_location)
        client.utils.sync_buffered_reasoner()
