import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

bridge = CvBridge()
cap = cv.VideoCapture(0)
if not cap.isOpened():
	print("Cannot open camara")
	exit()

while True:
	ret, frame = cap.read()

	if not ret:
		print("Can't receive frame (stream end?). Exiting ...")
		break

	# cv.imwrite('Grises.jpg', frame)
    # image_message = bridge.cv2_to_imgmsg(frame, encoding = "passthrough")

cap.release()

