#! /usr/bin/env python3
"""
.. module:: ontology_build.py
   :platform: Unix
   :synopsis: Code for initialization of the ontology  
.. moduleauthor:: Krishant Tharun
This script defines the structure of the finite state machine. All of its states and related transitions are initialized in this place.
It is specified how to behave for each state at each transition that is received in order to prevent the system from being stuck or encountering errors during transitions.
A helper entity is also constructed at the beginning of the execution and supplied as a parameter to each state to make it easier to use functions and shared variables in specific circumstances. This object
is a property of the applicable Class:mod:'helper'.
However, sharing the mutex that is necessary to easily access shared variables is the helper's main purpose. In order to achieve complete synchronization between the state and the reading and writing operations, just one mutex is really used in this implementation.
 
"""

import time 
import random
import rospkg
import re
import sys
import rospy
import roslib

from os.path import dirname, realpath
from Assignment import architecture_name_mapper as nm
from assignment.msg import Point
from armor_api.armor_client import ArmorClient

client = ArmorClient("armor_client", "ontology")

path = dirname(realpath(__file__))
path = path + "/../ontology_map/"

client.utils.load_ref_from_file(path + "topology.owl", "http://bnc/exp-rob-lab/2022-23", True, "PELLET", False, False)
client.utils.mount_on_ref()
client.utils.set_log_to_terminal(True)

def build_ontology_map():
    print("Building ontology map...")
	client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', 'E')
	client.manipulation.add_ind_to_class('E', 'LOCATION')
		
	client.manipulation.add_ind_to_class('C1', 'LOCATION')
	client.manipulation.add_ind_to_class('C2', 'LOCATION')
	client.manipulation.add_ind_to_class('R1', 'LOCATION')
	client.manipulation.add_ind_to_class('R2', 'LOCATION')
	client.manipulation.add_ind_to_class('R3', 'LOCATION')
	client.manipulation.add_ind_to_class('R4', 'LOCATION')
		
	client.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D5')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'E', 'D6')

	client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D1')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D2')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D5')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'C1', 'D7')

	client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D3')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D4')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D5')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'C2', 'D6')

	client.manipulation.add_objectprop_to_ind('hasDoor', 'R1', 'D1')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'R2', 'D2')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'R3', 'D3')
	client.manipulation.add_objectprop_to_ind('hasDoor', 'R4', 'D4')
    
    client.call('DISJOINT', 'IND', '', ['E','C1','C2','R1','R2','R3','R4','D1','D2','D3','D4','D5','D6','D7'])
    
    _actual_time = str(int(time.time()))
    
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R1', 'Long', _actual_time)
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R2', 'Long', _actual_time)
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R3', 'Long', _actual_time)
    client.manipulation.add_dataprop_to_ind('visitedAt', 'R4', 'Long', _actual_time)
    
    client.utils.apply_buffered_changes()
    client.utils.sync_buffered_reasoner()


    
    
    


    

    
    
    

    
