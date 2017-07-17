
#impor required libraries
import numpy as np # numpy for array manipulation
import cv2	#OpenCv
import cv2.cv as cv #Open Cv
import math
import serial 
import struct 
import time

import freenect # Kinect free library

import timeit

#ser = serial.Serial('/dev/ttyUSB0',57600)

# function to write a string in a frame
def draw_str(dst, (x, y), s):
    cv2.putText(dst, s, (x+1, y+1), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 0), thickness = 2, lineType=cv2.CV_AA)
    cv2.putText(dst, s, (x, y), cv2.FONT_HERSHEY_PLAIN, 1.0, (255, 255, 255), lineType=cv2.CV_AA)


#function to get RGB image from kinect
def get_video():
    array,_ = freenect.sync_get_video()
    array = cv2.cvtColor(array,cv2.COLOR_RGB2BGR)
    return array
    
#function to get depth image from kinect
def get_depth():
    array,_ = freenect.sync_get_depth()
    array = array.astype(np.uint8)
    return array

#function to get distances in cm, equation from OpenKinect    
def get_distance_St(depthRaw):
	depthM = 0.1236 * math.tan(depthRaw / 2842.5 + 1.1863)
	return depthM * 100

#function to cal real X from pixel and Z distance in centimeters
def calcXCM(pixelX, distZ):
	xCm = (pixelX - 320) * (distZ - 10) * 0.0021
	return xCm
	
#function to cal real Y from pixel and Z distance in centimeters	
def calcYCM(pixelY, distZ):
	yCm = (pixelY - 240) * (distZ - 10) * 0.0021
	return -yCm

# function to be passed as parameter in trackBars initialization
def nothing(x):
    pass

#function to border highlight
def border(frameGray):
	sobelx = cv2.Sobel(frameGray,cv2.CV_64F,1,0,ksize=3)
	sobelxAbs = cv2.convertScaleAbs(sobelx)
	sobely = cv2.Sobel(frameGray,cv2.CV_64F,0,1,ksize=3)
	sobelyAbs = cv2.convertScaleAbs(sobely)
	FrameBordas=cv2.addWeighted(sobelxAbs,0.5,sobelyAbs,0.5,0)
  	ret2, th2 = cv2.threshold(FrameBordas,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	return th2

#function to detect circles
def detectCircles(frametoprintcircles, fametofindcircles):
	circles = cv2.HoughCircles(fametofindcircles,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	
	if circles is not None:
		for i in circles[0,:]:
			#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
		
			# draw a circle around the object in the original image
			cv2.circle(frametoprintcircles,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(frametoprintcircles,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
	return frame

cv2.namedWindow("Original")
cv2.namedWindow("depth")

fourcc =  cv2.cv.CV_FOURCC(*'XVID')
video = cv2.VideoWriter("ImagemKinctRGB.avi", fourcc, 30, (640,480), 1)
video2 = cv2.VideoWriter("ImagemKinctDEPTH.avi", fourcc, 30, (320,240), 1)


while 1:
	
	frame = get_video() #get RGB image from kinect
	depth = get_depth() #get Depth image normalized from kinect, just to show
	
	depthOriginal,_ = freenect.sync_get_depth() # get 11 bit depth value from kinect
	
	#for i in range(nFrames):
	
	video.write(frame)
	video2.write(depth)
	#logData.write(depthOriginal)

	np.savetxt('DadosProfundidade', depthOriginal, delimiter='\n- ') 
	
	cv2.imshow('Original',frame)	
	cv2.imshow('depth',depth)

	key_pressed = cv2.waitKey(1)
logData.close()

