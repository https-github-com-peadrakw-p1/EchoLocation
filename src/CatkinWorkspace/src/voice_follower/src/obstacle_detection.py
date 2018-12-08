#!/usr/bin/env python

# Imports
import rospy
import cv2
import enum
import random
import array
import numpy as np
from std_msgs.msg import Float64

scanner_readings = array.array('i',(0 for i in range(0,90)))

class DistancesEnum(enum.IntEnum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2


class DirectionEnum(enum.IntEnum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2


def shutdown_sequence(cls_obj):
    # Do something
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


def scanCallBack(ros_data):
    global scanner_readings

    # Create a deep copy of all scan data
    data_copy = copy.deepcopy(list(ros_data.ranges))

    # Copy only the readings from -45 degrees to 45 degrees
    scanner_readings = data_copy[359-45:359] + (data_copy[0:45])

    # Change all values which are 0 to inf
    for i in range(0,len(scanner_readings)):
        if scanner_readings[i] < 0.01:
            scanner_readings[i] = float('inf')


class ObsticleDetection:
    def CheckForObsticles(self, readings):

        # Calculate the closest obsticle in my center view
        mid_point = int(len(readings) / 2)
        center_readings = copy.deepcopy(readings[mid_point - 10:mid_point + 10])
        center_sorted_readings = sorted(center_readings)
        center_threeSmallestReadings = [center_sorted_readings[0], center_sorted_readings[1], center_sorted_readings[2]] 
        center_closest_obsticle = np.nanmean(center_threeSmallestReadings);

        # Calculate the closest obsticle in all my view
        # sorted_readings = sorted(copy.deepcopy(readings))
        # threeSmallestReadings = [sorted_readings[0], sorted_readings[1], sorted_readings[2]] 
        # all_closest_obsticle = np.nanmean(threeSmallestReadings);
        all_closest_obsticle = np.min(readings)

        if all_closest_obsticle > 1.5:
            return DistancesEnum.FAR
        elif center_closest_obsticle < 0.5:
            return DistancesEnum.CLOSE
        else:
            return DistancesEnum.MEDIUM

    def CheckSpace(self, readings):
        object_direction = 0

        for i in range(0,45):
            if readings[i] < 1:
                object_direction = 45 - i
                break 
            if readings[-i] < 1:
                object_direction = -45 + i
                break

        return object_direction


if __name__=="__main__":
    global scanner_readings

    # Start the ros node
    rospy.init_node('obstacle_detection')
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    subscriber = rospy.Subscriber('/scan', LaserScan, scanCallBack)
    
    # Setting the rate
    set_rate = 20
    rate = rospy.Rate(set_rate)

    # Allow Ros to catch up
    time.sleep(1)

    # Creating all the robot objects
    OD_Obj = ObsticleDetection()

    # Remember the state of the robot
    current_readings = DistancesEnum.FAR

    # Ros Loop
    while not rospy.is_shutdown():

        # Check for obstacles
        obsticle_distance = OD_Obj.CheckForObsticles(scanner_readings)
        turn_direction = OD_Obj.CheckSpace(scanner_readings)
        print("Turn Direction: " + str(turn_direction))

        # Sleep
        rate.sleep()




















