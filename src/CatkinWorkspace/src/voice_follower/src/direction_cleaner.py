#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

# Declare the global reading 
global dirty_reading
dirty_reading = [0,0]

# On shutdown run this command
def shutdown_sequence():
    # Do something
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


def dirtyDirectionCallBack(ros_data):
    global dirty_reading

    # Make the previous reading the current reading
    dirty_reading[1] = dirty_reading[0]

    # Update the current reading
    dirty_reading[0] = ros_data


class KalmanFilter:

    def __init__(self):
	    # Declare if you want to use any variables here
	    self.var1 = 0

    def CheckForObsticles(self, readings):
    	# Do some function here
    	rospy.loginfo(str(rospy.get_name()) + ": " + str(self.var1))
    	rospy.loginfo(str(rospy.get_name()) + ": " + str(readings[0]) + str(readings[0]))
    	return readings * self.var1


if __name__=="__main__":
	global dirty_reading

    # Start the ros node
    rospy.init_node('direction_cleaner_node')

    # On shutdown run the following function
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    pub_dir = rospy.Publisher('/clean_direction', Float64, queue_size=10)
    sub_dir = rospy.Subscriber('/raw_direction', Float64, dirtyDirectionCallBack)
    
    # Setting the rate
    set_rate = 20
    rate = rospy.Rate(set_rate)

    # Creating the filter object
    KF_Obj = KalmanFilter()

    # The main ros loop
    while not rospy.is_shutdown():

        # Call the Kalman Filter
        KF_Obj.CheckForObsticles(dirty_reading)

        # Sleep
        rate.sleep()