# EchoLocation
import matplotlib.pyplot as plt
import numpy as np
import cv2
import math
import copy

# Open a camera device
cap = cv2.VideoCapture(0)

# Capture a single frame from the camera
ret, frame = cap.read()

# Calculate the length of the frame
x_length = frame.shape[1]
y_length = -1 * frame.shape[0]

# Find the center of the object
center = [x_length / 2.0, y_length / 2.0]

# Display the graph
plt.clf()
plt.xlim((0,x_length))
plt.ylim((0,y_length))
plt.ylabel('Position Y')
plt.xlabel('Position X')
plt.title("Angle: 0")
plt.plot([center[0],center[0]],[center[1],center[1] - 150],'o--')
plt.pause(0.000001)

while(True):
	# Capture frame-by-frame
	ret, orig_frame = cap.read()

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
		deltax = center[0] - av_x
		deltay = center[1] - av_y
		angle = math.atan2(deltay, deltax)

		# Offset to make top of image 0 degrees
		angle = angle + math.pi/2.0
		angle = -1 * math.atan2(math.sin(angle), math.cos(angle))

		# Display how the angle is calculated
		plt.clf()
		plt.xlim((0, x_length))
		plt.ylim((y_length, 0))
		plt.ylabel('Position Y')
		plt.xlabel('Position X')
		plt.title("Angle: " + str(math.degrees(angle)))
		plt.plot([center[0],av_x], [center[1],av_y], 'o--')
		plt.pause(0.000001)

		# Display the position information
		print("Center Point: " + str(center))
		print("Center Light: " + str(av_w_pixel))

		# Display the angle
		print("Angle: " + str(math.degrees(angle)))
		print("==============================================")


	# Display the images
	cv2.imshow('Original Image',orig_frame)
	cv2.imshow('Removed Blue Image',frame)
	cv2.imshow('Blurred Image',dst4)
	cv2.imshow('Grayscale Image',im_gray)
	cv2.imshow('Final Binary Image',im_bw)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()