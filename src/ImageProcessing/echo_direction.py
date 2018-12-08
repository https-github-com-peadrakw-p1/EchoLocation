import numpy as np
import cv2

# Open a camera device
cap = cv2.VideoCapture(0)

while(True):
	# Capture frame-by-frame
	ret, frame = cap.read()

	# Remove the blue from the image
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

	box1 = im_bw[0:240,0:320]
	box2 = im_bw[0:240,320:640]
	box3 = im_bw[240:480,0:320]
	box4 = im_bw[240:480,320:640]

	# Find which quater it is in
	quadrents = [np.mean(box1), np.mean(box2), np.mean(box3), np.mean(box4)]
	quater = np.argmax(quadrents)
	print(quater)
	
	# Display the image in binary
	cv2.imshow('frame',im_bw)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()