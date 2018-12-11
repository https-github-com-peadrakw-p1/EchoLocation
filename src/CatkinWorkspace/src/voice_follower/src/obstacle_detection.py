#!/usr/bin/env python

# Imports
import rospy
import cv2
import copy
import enum
import random
import time
import array
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from voice_follower.msg import ObstacleData
from std_msgs.msg import Int8

global scanner_readings

class DistancesEnum(enum.IntEnum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2


class DirectionEnum(enum.IntEnum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = -1


def shutdown_sequence():
    # Do something
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


def scanCallBack(ros_data):
    global scanner_readings

    # Make a deep copy of the data so that we dont change scanner readings
    data_copy = copy.deepcopy(list(ros_data.ranges))

    # Copy only the center readings
    mid_point = int(len(data_copy) / 2)
    scanner_readings = data_copy[mid_point - 40:mid_point + 40]

    # Change all values which are 0 to inf
    for i in range(0,len(scanner_readings)):
        if math.isnan(scanner_readings[i]):
            scanner_readings[i] = float('inf')



class ObsticleDetection:

    def CheckForObsticles(self, readings):

        # Calculate the closest obsticle in my center view
        mid_point = int(len(readings) / 2)
        center_readings = copy.deepcopy(readings)
        center_sorted_readings = sorted(center_readings)
        center_threeSmallestReadings = [center_sorted_readings[0], center_sorted_readings[1], center_sorted_readings[2]] 
        center_closest_obsticle = np.nanmean(center_threeSmallestReadings);

        # Find the minimum value of all readings
        all_closest_obsticle = np.min(readings)


        # Return how far away an obstical is
        if all_closest_obsticle > 0.9:
            return DistancesEnum.FAR
        elif center_closest_obsticle < 0.8:
            return DistancesEnum.CLOSE
        else:
            return DistancesEnum.MEDIUM

    def CheckSpace(self, readings):
        # Initilize the object direction to straight ahead
        object_direction = DirectionEnum.STRAIGHT

        # Loop through the array
        for i in range(0, len(readings)/2):
            # If its in the first half turn left
            if (readings[i] < 1) or (readings[i] == float('inf')):
                object_direction = DirectionEnum.LEFT
                break 
            # If its in the second half turn right
            if (readings[-i] < 1) or (readings[i] == float('inf')):
                object_direction = DirectionEnum.RIGHT
                break

        return object_direction


if __name__=="__main__":
    global scanner_readings

    # Start the ros node
    rospy.init_node('obstacle_detection')
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    sub_scn = rospy.Subscriber('/scan', LaserScan, scanCallBack)
    pub_odt = rospy.Publisher('/obstacle_info', ObstacleData, queue_size=10)

    # Initilize the scanner readings (incase you never get any)
    scanner_readings = array.array('i',(0 for i in range(0,90)))

    # Setting the rate
    set_rate = 20
    rate = rospy.Rate(set_rate)

    # Allow Ros to catch up
    time.sleep(1)

    # Creating all the robot objects
    OD_Obj = ObsticleDetection()

    # Create the message we are going to use to send information
    obs_data = ObstacleData()
  
    # Ros Loop
    while not rospy.is_shutdown():

        # Check for obstacles
        o_distance = OD_Obj.CheckForObsticles(scanner_readings)
        t_direction = OD_Obj.CheckSpace(scanner_readings)

        # Print out the readings
        rospy.loginfo(str(rospy.get_name()) + ": Obstacle Distance - " + str(o_distance))
        rospy.loginfo(str(rospy.get_name()) + ": Turn Direction: " + str(t_direction))

        # Create the message for publishing
        obs_data.obstacle_distance = o_distance
        obs_data.turn_direction = t_direction

        # Publish the message
        pub_odt.publish(obs_data)

        # Sleep
        rate.sleep()




















