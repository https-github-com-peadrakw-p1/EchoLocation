#!/usr/bin/env python

import rospy
import copy
import enum
import math
import time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Empty

from voice_follower.msg import ObstacleData

global obsticle_distance
global turn_direction
global goal_heading
global vel
global pos


def shutdown_sequence():
    # Create a stop message for the robot
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


class DistancesEnum(enum.IntEnum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2


class DirectionEnum(enum.IntEnum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = -1


def obstacleCallBack(ros_data):
    global obsticle_distance
    global turn_direction
    
    # Save the incomming ros data
    obsticle_distance = ros_data.obstacle_distance
    turn_direction = ros_data.turn_direction


def directionCallBack(ros_data):
    global goal_heading

    # Save the incomming ros data
    goal_heading = ros_data.data  

def processOdom(ros_data):
    global vel
    global pos

    # Get the position data from the odom
    pos[0] = ros_data.pose.pose.position.x
    pos[1] = ros_data.pose.pose.position.y 

    # Get the velocity data from the odom
    vel = ros_data.twist.twist.linear.x


class RobotBehaviourSensors:
    global vel
    global pos

    def __init__(self, rate_in):
        # Inititilize the current goal and position
        self.current_position = [0,0]

        # Previous Positions is an array of 2 values (can be increased later)
        self.previous_position = []
        self.previous_list_length = 2
        for i in range(0,self.previous_list_length):
            self.previous_position.append(self.current_position)

        # Calculate the time between position updates
        self.dt = 1.0 / rate_in


    def updatePosition(self):
        # Make a deep copy of the update
        current_position_dc = copy.deepcopy(pos)

        # Set the current position (make sure its a deep copy, so that chaning current_position does not change all values in the previous array)
        self.current_position = copy.deepcopy(current_position_dc)

        # Save the position to the previous position array
        self.previous_position.insert(0, current_position_dc)
        if len(self.previous_position) > self.previous_list_length:
            # Remove the oldest previous position element
            self.previous_position.pop()

    def GetRobotHeading(self):
        # Calculate the change in distance in both X and Y
        deltaX = self.current_position[0] - self.previous_position[-1][0]
        deltaY = self.current_position[1] - self.previous_position[-1][1]

        # Calculate the angle between the two points
        current_heading = math.atan2(deltaY, deltaX)

        print("CURRENT HEADING: " + str(current_heading))

        # Return the current heading
        return current_heading

    def GetRobotVelocity(self):
        # Return the read velocity
        return vel


class Planner:

    def __init__(self, rate):
        # PID values
        self.ang_prev_err = 0
        self.ang_integral = 0
        self.vel_prev_err = 0
        self.vel_integral = 0

        # PID parameters
        self.ang_Kp = 1.0
        self.ang_Ki = 0.0
        self.ang_Kd = 0.0
        self.vel_Kp = 0.6
        self.vel_Ki = 0.2
        self.vel_Kd = 0.0

        # Setting the rate
        self.dt = 1.0/rate

    def GetAngularVelocity(self, object_distance, object_side, robot_heading, desired_heading):

        # The robots desired heading is calculated 
        if object_distance == DistancesEnum.MEDIUM:
            desired_heading = desired_heading + (object_side * (math.pi/4.0))

        if object_distance == DistancesEnum.FAR:
            desired_heading = desired_heading

        # Angular velocity heading error
        error = desired_heading - robot_heading
        error = math.atan2(math.sin(error), math.cos(error))

        # Angular velocity PID
        self.ang_integral = self.ang_integral + (error * self.dt)
        derivate = (error - self.ang_prev_err) / self.dt
        self.ang_prev_err = error

        # Calculate the angle
        PID_angle = self.ang_Kp*error + self.ang_Ki*self.ang_integral + self.ang_Kd*derivate
        anglular_velocity = PID_angle

        return anglular_velocity

    def GetLinearVelocity(self, object_distance, robot_speed):

        # Desired speed is a function of distance
        desired_speed = 0
        if object_distance == DistancesEnum.CLOSE:
            desired_speed = 0.01
        elif object_distance == DistancesEnum.MEDIUM:
            desired_speed = 0.2
        else:
            desired_speed = 0.35

        print(desired_speed)

        # Velocity error
        error = desired_speed - robot_speed 
        error = math.atan2(math.sin(error), math.cos(error))

        # Linaer velocity PID
        self.vel_integral = self.vel_integral + (error * self.dt)
        derivate = (error - self.vel_prev_err) / self.dt
        self.vel_prev_err = error

        # Calculate speed
        PID_speed = self.vel_Kp*error + self.vel_Ki*self.vel_integral + self.vel_Kd*derivate
        linear_velocity = PID_speed

        return linear_velocity


if __name__=="__main__":
    global obsticle_distance
    global turn_direction
    global goal_heading

    # Start the ros node
    rospy.init_node('planner_node')
    rospy.on_shutdown(shutdown_sequence)

    # Initlize the global variables
    obsticle_distance = 0
    turn_direction = 0
    goal_heading = 0
    vel = 0
    pos = [0,0]

    # Subscribers and publishers
    subscriber = rospy.Subscriber('/obstacle_info', ObstacleData, obstacleCallBack)
    subscriber = rospy.Subscriber('/final_direction', Float64, directionCallBack)
    subscriber = rospy.Subscriber('/odom', Odometry, processOdom)
    pub_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
    reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)

    # Reset the odometry
    timer = time.time()
    while time.time() - timer < 0.25:
        reset_odom.publish(Empty())
    
    # Setting the rate
    set_rate = 20
    rate = rospy.Rate(set_rate)

    # Allow Ros to catch up
    time.sleep(1)

    # Creating all the robot objects
    PN_Obj = Planner(set_rate)
    RB_Obj = RobotBehaviourSensors(set_rate)

    # ROS Loop
    while not rospy.is_shutdown():

        # Get the robot and goal headings
        RB_Obj.updatePosition()
        current_heading = RB_Obj.GetRobotHeading()
        current_speed = RB_Obj.GetRobotVelocity()

        # Do this to fix the frame difference
        current_heading = 0

        # Scale the goal angle such that it doesnt turn as hard
        scaled_goal_angle = copy.deepcopy(goal_heading)
        scaled_goal_angle = scaled_goal_angle / (math.pi/2)

        print("Velocity: " + '\t' + str(current_speed))
        print("Heading: " + '\t' + str(math.degrees(current_heading)))
        print("Goal Heading: " + '\t' + str(math.degrees(scaled_goal_angle)))

        # Get the next move
        required_angular_velocity = PN_Obj.GetAngularVelocity(obsticle_distance, turn_direction, current_heading, scaled_goal_angle)
        required_linear_velocity = PN_Obj.GetLinearVelocity(obsticle_distance, current_speed)

        # Set the state of the robot 
        if -0.5 < required_angular_velocity < 0.5:
            current_state = DirectionEnum.STRAIGHT
        if required_angular_velocity > 0.5:
            current_state = DirectionEnum.LEFT
        if required_angular_velocity < -0.5:
            current_state = DirectionEnum.RIGHT

        # Publish the message
        twist = Twist()
        twist.linear.x = 0.3
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = required_angular_velocity
        pub_vel.publish(twist)

        # Sleep
        rate.sleep()