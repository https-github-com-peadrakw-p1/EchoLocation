#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64

# Declare the global reading 
global raw_reading

# On shutdown run this command
def shutdown_sequence():
    # Do something
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


def RawDirectionCallBack(ros_data):
    global raw_reading

    # Save the data
    raw_reading = ros_data.data


class KalmanFilter:
    # On class creation
    def __init__(self, rate):

        self.delta_t = 1.0/rate

        self.A = np.array([1])
        self.B = np.array([self.delta_t])
        self.H = np.array([1])
        self.Q = np.array([pow(self.delta_t,2)])

        self.R = np.array([1])
        self.x_prev = np.array([0]) # initial angle
        self.P_prev = np.array([self.delta_t]) #initial p

    
    def get_angle(self, z):
        #Prediction
        u = self.delta_t * (z - self.x_prev)
        self.x_pred = self.A * self.x_prev + self.B * u
        self.P_pred = self.A * self.P_prev * self.A.T + self.Q
        
        #Correction
        Kk = self.P_pred * self.H.T * (self.H * self.P_pred * self.H.T + self.R)
        x = self.x_pred + Kk * (z - self.H * self.x_pred)
        
        self.x_prev = x
        self.P_prev = (1 - Kk * self.H) * self.P_pred
        
        return x


if __name__=="__main__":
    global raw_reading

    # Start the ros node
    rospy.init_node('direction_filter_node')

    # On shutdown run the following function
    rospy.on_shutdown(shutdown_sequence)

    # Initilize the raw reading
    raw_reading = 0

    # Subscribers and publishers
    pub_dir = rospy.Publisher('/final_direction', Float64, queue_size=10)
    sub_dir = rospy.Subscriber('/raw_direction', Float64, RawDirectionCallBack)

    # Setting the rate
    set_rate = 4
    rate = rospy.Rate(set_rate)

    # Creating the filter object
    KF_Obj = KalmanFilter(set_rate)

    # Creating the final direction message
    final_dir = Float64()

    # The main ros loop
    while not rospy.is_shutdown():

        # Call the Kalman Filter
        final_dir.data = KF_Obj.get_angle(raw_reading)

        # Publish the direction
        pub_dir.publish(final_dir)

        # Sleep
        rate.sleep()