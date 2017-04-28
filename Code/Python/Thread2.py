import time
import picamera
import numpy as np
import cv2

import RPi.GPIO as gp 

gp.setwarnings(False)
gp.setmode(gp.BOARD)

JUMPER_SETTINGS = ‘B’

# Jumper Pin Assignment
IVJP = {'A': (11,12), 'C': (21,22), 'B': (15,16), 'D': (23,24)}
pins = list(reduce(lamda x,y: x+y, IVJP.values()))
pins.sort
DIVJP = {i+1 : x for i, in in enumerate(pins)}
f1Pin, f2Pin = DIVJP[JUMPER_SETTINGS]
ePin = 7

gp.setup(f1Pin, gp.OUT)
gp.setup(f2Pin, gp.OUT)
gp.setup(ePin, gp.OUT)

gp.output(f1Pin, False)
gp.output(f2Pin, False)
gp.output(ePin, False)

frame = 2
cam = 1
count = 0

def cam_change():
	global cam_change
	gp.setmode(gp.BOARD)
	if cam ==1:
		gp.output(ePin, False)
		gp.output(f1Pin, True)
		gp.output(f2Pin, False)
	
	elif cam ==2 :
		gp.output(ePin, True)
		gp.output(f1Pin, False)
		gp.output(f2Pin, True)

	elif cam ==3 :
		gp.output(ePin, False)
		gp.output(f1Pin, False)
		gp.output(f2Pin, True)

	elif cam. == 4:
		gp.output(ePin, True)
		gp.output(f1Pin, True)
		gp.output(f2Pin, True)

	cam += 2
	if cam > 4:
		cam = 1

with picamera.PiCamera() as camera:
	camera.resolution = (2048, 1536)
	camera.framerate = 90
	rawCapture = PiRGBArray(camera)
	
	time.sleep(0.1) #camera initialization
	camera.capture(rawCapture, format="bgr")
	image1 = rawCapture.array
	cam_change()

	time.sleep(0.1) #camera initialization
	camera.capture(rawCapture, format="bgr")
	image2 = rawCapture.array

	window_size = 17
	min_disp =0
	num_disp = 32
	stereo = cv2.StereoSGBM_create(minDisparity = min_disp, numDisparities = num_disp, 	blockSize = 16)

	dis = stereo.compute(img1, img2).astype(np.float32) / 16.0
	
	h, w = image1.shape[:2]
	f = .36 //this depends on the camera calibration
	Q = np.float32([[1, 0, 0, -.5*w],
			[0, -1, 0, 0.5*h],
			[0, 0, 0, -f],
			[0, 0, 1, 0]]) //this depends on the camera calibration
	points = cv2.reprojectImageTo3D(disc,0)
	colors = cv2.cvtColor(imgage1, cv2.COLOR_BGR2RGB)
	mask = display >display.min()
	out_points = points[mask]
	out_colors = colors[mask]

	return out_points, out_colors
