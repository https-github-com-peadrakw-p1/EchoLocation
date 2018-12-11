#!/usr/bin/env python

# Imports
import rospy
import cv2
import copy
import math
import numpy as np
from std_msgs.msg import Float64


# On shutdown run this command
def shutdown_sequence():
    # Do something
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down ")


class CameraProcessing:
    # On class creation
    def __init__(self):
        # Open a camera device
        self.cap = cv2.VideoCapture(1)

        # Capture a single frame from the camera
        ret, frame = self.cap.read()

        # Calculate the length of the frame
        self.x_length = frame.shape[1]
        self.y_length = -1 * frame.shape[0]

        # Find the center of the object
        self.center = [self.x_length / 2.0, self.y_length / 2.0]

        # Declare the final result
        self.angle = 0

    def ProcessImage(self):
        # Capture frame-by-frame
        ret, orig_frame = self.cap.read()

        # Remove the blue from the image
        frame = copy.deepcopy(orig_frame)
        frame[:,:,0] = 0

        # Blur the image
        kernel = np.ones((5,5),np.float32)/25
        dst1 = cv2.filter2D(frame,-1,kernel)
        dst2 = cv2.filter2D(dst1,-1,kernel)
        dst3 = cv2.filter2D(dst2,-1,kernel)
        dst4 = cv2.filter2D(dst3,-1,kernel)

        # Our operations on the frame come here
        im_gray = cv2.cvtColor(dst4, cv2.COLOR_BGR2GRAY)

        # Get the brightest point using a threshold
        thresh = 185
        (thresh, im_bw) = cv2.threshold(im_gray, thresh, 255, cv2.THRESH_BINARY)

        # Find the x and y positions of all the white pixels
        position_white_pixels = np.argwhere(im_bw)
        av_w_pixel = np.mean(position_white_pixels, axis=0)

        # X is height, Y is width
        av_x = av_w_pixel[1]
        av_y = -1 * av_w_pixel[0]

        # Only while we are getting useful informaiton
        if (math.isnan(av_x) == False) and (math.isnan(av_y) == False):

            # Calculate the exact angle
            deltax = self.center[0] - av_x
            deltay = self.center[1] - av_y
            self.angle = math.atan2(deltay, deltax)

            # Offset to make top of image 0 degrees
            self.angle = self.angle + math.pi/2.0
            self.angle = math.atan2(math.sin(self.angle), math.cos(self.angle))

            # Display the position information
            rospy.loginfo(str(rospy.get_name()) + ": Center Point: " + str(self.center))
            rospy.loginfo(str(rospy.get_name()) + ": Center Light: " + str(av_w_pixel))

            # Display the angle
            rospy.loginfo(str(rospy.get_name()) + ": Angle: " + str(math.degrees(self.angle)))


        return self.angle

    def DestoryObject(self):
        # When everything done, release the capture
        self.cap.release()
        cv2.destroyAllWindows()


if __name__=="__main__":
    # Start the ros node
    rospy.init_node('amazon_echo_reader')

    # On shutdown run the following function
    rospy.on_shutdown(shutdown_sequence)

    # Subscribers and publishers
    pub_dir = rospy.Publisher('/raw_direction', Float64, queue_size=10)
    
    # Setting the rate
    set_rate = 6
    rate = rospy.Rate(set_rate)

    # Creating the filter object
    CP_Obj = CameraProcessing()

    # The main ros loop
    while not rospy.is_shutdown():

        # Process an image
        direction = CP_Obj.ProcessImage()

        # Publish the direction
        pub_dir.publish(Float64(direction))

        # Sleep
        rate.sleep()

    # Destroy the camera object
    CP_Obj.DestoryObject()