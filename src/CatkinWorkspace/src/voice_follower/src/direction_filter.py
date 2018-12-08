#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

# Declare the global reading 
raw_reading = [0,0]

# On shutdown run this command
def shutdown_sequence():
    # Do something
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


def RawDirectionCallBack(ros_data):
    global raw_reading

    # Make the previous reading the current reading
    raw_reading[1] = raw_reading[0]

    # Update the current reading
    raw_reading[0] = ros_data.data


class KalmanFilter:

    def __init__(self):
	    # Declare if you want to use any variables here
	    self.var1 = 0

    def FunctionCall(self, readings):
    	# Do some function here
    	rospy.loginfo(str(rospy.get_name()) + ": " + str(self.var1))
    	rospy.loginfo(str(rospy.get_name()) + ": " + str(readings[0]) + str(readings[0]))
    	return readings[0] * self.var1


if __name__=="__main__":
    global raw_reading

    # Start the ros node
    rospy.init_node('direction_filter_node')

    # On shutdown run the following function
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    pub_dir = rospy.Publisher('/final_direction', Float64, queue_size=10)
    sub_dir = rospy.Subscriber('/raw_direction', Float64, RawDirectionCallBack)

    # Setting the rate
    set_rate = 2
    rate = rospy.Rate(set_rate)

    # Creating the filter object
    KF_Obj = KalmanFilter()

    # Creating the final direction message
    final_dir = Float64()

    # The main ros loop
    while not rospy.is_shutdown():

        # Call the Kalman Filter
        final_dir.data = KF_Obj.FunctionCall(raw_reading)

        # Publish the direction
        pub_dir.publish(final_dir)

        # Sleep
        rate.sleep()