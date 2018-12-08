#!/usr/bin/env python

import rospy


def shutdown_sequence():
    # Create a stop message for the robot
    twist = Twist()
    twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
    twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
    pub_vel.publish(twist)


class DistancesEnum(enum.IntEnum):
    CLOSE = 0
    MEDIUM = 1
    FAR = 2


class DirectionEnum(enum.IntEnum):
    STRAIGHT = 0
    LEFT = 1
    RIGHT = 2


class Planner:

    def __init__(self, rate):
        # PID values
        self.ang_prev_err = 0
        self.ang_integral = 0
        self.vel_prev_err = 0
        self.vel_integral = 0

        # PID parameters
        self.ang_Kp = 1.1
        self.ang_Ki = 0.0
        self.ang_Kd = 0.0
        self.vel_Kp = 0.5
        self.vel_Ki = 0.1
        self.vel_Kd = 0.0

        # Setting the rate
        self.dt = 1.0/rate

    def GetAngularVelocity(self, object_distance, object_side, robot_heading, desired_heading):

        # The robots desired heading is calculated 
        if object_distance == DistancesEnum.MEDIUM:
            desired_heading = desired_heading + object_side

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

    def GetLinearVelocity(self, object_distance, goal_distance, robot_speed):

        # Desired speed is a function of distance
        desired_speed = min((goal_distance + 0.1 /2),1) * 0.5

        # If the object is close, stop the robot
        if object_distance == DistancesEnum.CLOSE:
            # Stop the robot
            desired_speed = 0

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

    # Start the ros node
    rospy.init_node('planner_node')
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    subscriber = rospy.Subscriber('/obstacle_info', Int, scanCallBack)
    subscriber = rospy.Subscriber('/final_direction', Float64, scanCallBack)
    
    # Setting the rate
    set_rate = 20
    rate = rospy.Rate(set_rate)

    # Allow Ros to catch up
    time.sleep(1)

    # Creating all the robot objects
    PN_Obj = Planner(set_rate)

    # ROS Loop
    while not rospy.is_shutdown():

        print("Velocity: " + '\t' + str(current_speed))
        print("Heading: " + '\t' + str(current_heading))

        # Get the next move
        required_angular_velocity = PN_Obj.GetAngularVelocity(obsticle_distance, turn_direction, current_heading, goal_heading)
        required_linear_velocity = PN_Obj.GetLinearVelocity(obsticle_distance, distance_to_goal, current_speed)

        # Set the state of the robot 
        if -0.5 < required_angular_velocity < 0.5:
            current_state = DirectionEnum.STRAIGHT
        if required_angular_velocity > 0.5:
            current_state = DirectionEnum.LEFT
        if required_angular_velocity < -0.5:
            current_state = DirectionEnum.RIGHT

        # Publish the message
        twist = Twist()
        twist.linear.x = 1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = required_angular_velocity
        pub_vel.publish(twist)

        # Sleep
        rate.sleep()