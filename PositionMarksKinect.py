#################################################################################
#
#	Ball tracking with Kinect - X,Y,Z
#
#	Description: Program to detect the position of a ball in space using the 
#				 Kinect sensor using the OpenCv(Computer Vision Library) and
#				 freenect(Open Source Kinect library).
#
# 	Date: 09/2016
#   Authors: Gustavo Carlos, Paulo Custodio
#		
#   LRVA - Laboratorio de Robotica e Veiculos Autonomos
################################################################################


#Values for Red ball
rhmin = 169
rhmax = 189	
rvmin = 160
rvmax = 190
rsmin = 102
rsmax = 139

#Values for Yellow ball
yhmin = 19
yhmax = 32
yvmin = 216
yvmax = 242
ysmin = 132
ysmax = 238

#Values for blue ball
bhmin = 106
bhmax = 124
bvmin = 122
bvmax = 190
bsmin = 61
bsmax = 111

verificaR=0
verificaY=0
verificaB=0
elapsedTime=0

#param1=80
#param2=30
mindist=15
minRadius=2

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

ser = serial.Serial('/dev/ttyUSB0',57600)

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

#vectors to use in openCv functions
kernel = np.ones((5,5),np.uint8)
kernel2 = np.ones((20,20),np.uint8)

#Windows to show images
cv2.namedWindow("Original")
#Vermelha
cv2.namedWindow('rHueAdj')
cv2.namedWindow("rSatAdj")
cv2.namedWindow("rValAdj")
#amarela
cv2.namedWindow('yHueAdj')
cv2.namedWindow("ySatAdj")
cv2.namedWindow("yValAdj")
#azul
cv2.namedWindow('bHueAdj')
cv2.namedWindow("bSatAdj")
cv2.namedWindow("bValAdj")

cv2.namedWindow("Depth")
cv2.namedWindow("Bordas")


#create track bars to adjust H, S and V of images

#vermelha
cv2.createTrackbar('hmin', 'rHueAdj',0,255,nothing)
cv2.createTrackbar('hmax', 'rHueAdj',0,255,nothing)
cv2.createTrackbar('smin', 'rSatAdj',0,255,nothing)
cv2.createTrackbar('smax', 'rSatAdj',255,255,nothing)
cv2.createTrackbar('vmin', 'rValAdj',0,255,nothing)
cv2.createTrackbar('vmax', 'rValAdj',0,255,nothing)
cv2.setTrackbarPos('hmin', 'rHueAdj', rhmin)
cv2.setTrackbarPos('hmax', 'rHueAdj', rhmax)
cv2.setTrackbarPos('smin', 'rSatAdj', rsmin)
cv2.setTrackbarPos('smax', 'rSatAdj', rsmax)
cv2.setTrackbarPos('vmin', 'rValAdj', rvmin)
cv2.setTrackbarPos('vmax', 'rValAdj', rvmax)
#amarela
cv2.createTrackbar('hmin', 'yHueAdj',0,255,nothing)
cv2.createTrackbar('hmax', 'yHueAdj',0,255,nothing)
cv2.createTrackbar('smin', 'ySatAdj',0,255,nothing)
cv2.createTrackbar('smax', 'ySatAdj',255,255,nothing)
cv2.createTrackbar('vmin', 'yValAdj',0,255,nothing)
cv2.createTrackbar('vmax', 'yValAdj',0,255,nothing)
cv2.setTrackbarPos('hmin', 'yHueAdj', yhmin)
cv2.setTrackbarPos('hmax', 'yHueAdj', yhmax)
cv2.setTrackbarPos('smin', 'ySatAdj', ysmin)
cv2.setTrackbarPos('smax', 'ySatAdj', ysmax)
cv2.setTrackbarPos('vmin', 'yValAdj', yvmin)
cv2.setTrackbarPos('vmax', 'yValAdj', yvmax)
#azul
cv2.createTrackbar('hmin', 'bHueAdj',0,255,nothing)
cv2.createTrackbar('hmax', 'bHueAdj',0,255,nothing)
cv2.createTrackbar('smin', 'bSatAdj',0,255,nothing)
cv2.createTrackbar('smax', 'bSatAdj',255,255,nothing)
cv2.createTrackbar('vmin', 'bValAdj',0,255,nothing)
cv2.createTrackbar('vmax', 'bValAdj',0,255,nothing)
cv2.setTrackbarPos('hmin', 'bHueAdj', bhmin)
cv2.setTrackbarPos('hmax', 'bHueAdj', bhmax)
cv2.setTrackbarPos('smin', 'bSatAdj', bsmin)
cv2.setTrackbarPos('smax', 'bSatAdj', bsmax)
cv2.setTrackbarPos('vmin', 'bValAdj', bvmin)
cv2.setTrackbarPos('vmax', 'bValAdj', bvmax)

logData = open('DadosRastreioRobo.txt', 'w')


countData = 0

while 1:
	#init = time.time()
	startTime = timeit.default_timer()
	
	
	frame = get_video() #get RGB image from kinect
	depth = get_depth() #get Depth image normalized from kinect, just to show
	
	depthOriginal,_ = freenect.sync_get_depth() # get 11 bit depth value from kinect

	frame2=cv2.GaussianBlur(frame,(3,3), 0);
	
	#convert to gray
	imgGray=cv2.cvtColor( frame2, cv2.COLOR_BGR2GRAY);
	imgRGB=cv2.cvtColor( imgGray, cv2.COLOR_GRAY2BGR);
	
	imgGray=border(imgGray)
	frame=detectCircles(imgRGB,imgGray)

	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #convert RGB image to HSV domain
	
	#get hue, sat and val separately from HSV image
	hue,sat,val = cv2.split(hsv)
	
	# get values min and max from trackBars 
	#vermelha
	rhmn = cv2.getTrackbarPos('hmin','rHueAdj') #144#
	rhmx = cv2.getTrackbarPos('hmax','rHueAdj') #180#
	rsmn = cv2.getTrackbarPos('smin','rSatAdj') #196#
	rsmx = cv2.getTrackbarPos('smax','rSatAdj') #255#
	rvmn = cv2.getTrackbarPos('vmin','rValAdj') #131#
	rvmx = cv2.getTrackbarPos('vmax','rValAdj') #191#
	#amarela
	yhmn = cv2.getTrackbarPos('hmin','yHueAdj') #144#
	yhmx = cv2.getTrackbarPos('hmax','yHueAdj') #180#
	ysmn = cv2.getTrackbarPos('smin','ySatAdj') #196#
	ysmx = cv2.getTrackbarPos('smax','ySatAdj') #255#
	yvmn = cv2.getTrackbarPos('vmin','yValAdj') #131#
	yvmx = cv2.getTrackbarPos('vmax','yValAdj') #191#
	#azul	
	bhmn = cv2.getTrackbarPos('hmin','bHueAdj') #144#
	bhmx = cv2.getTrackbarPos('hmax','bHueAdj') #180#
	bsmn = cv2.getTrackbarPos('smin','bSatAdj') #196#
	bsmx = cv2.getTrackbarPos('smax','bSatAdj') #255#
	bvmn = cv2.getTrackbarPos('vmin','bValAdj') #131#
	bvmx = cv2.getTrackbarPos('vmax','bValAdj') #191#

	# Apply thresholding
	#vermelha
	rhthresh = cv2.inRange(np.array(hue),np.array(rhmn),np.array(rhmx))
	rsthresh = cv2.inRange(np.array(sat),np.array(rsmn),np.array(rsmx))
	rvthresh = cv2.inRange(np.array(val),np.array(rvmn),np.array(rvmx))
	#amarela
	yhthresh = cv2.inRange(np.array(hue),np.array(yhmn),np.array(yhmx))
	ysthresh = cv2.inRange(np.array(sat),np.array(ysmn),np.array(ysmx))
	yvthresh = cv2.inRange(np.array(val),np.array(yvmn),np.array(yvmx))
	#azul
	bhthresh = cv2.inRange(np.array(hue),np.array(bhmn),np.array(bhmx))
	bsthresh = cv2.inRange(np.array(sat),np.array(bsmn),np.array(bsmx))
	bvthresh = cv2.inRange(np.array(val),np.array(bvmn),np.array(bvmx))

	# AND h s and v
	#vermelha
	rtracking = cv2.bitwise_and(rhthresh,cv2.bitwise_and(rsthresh,rvthresh))
	rtracking = border(rtracking)
	#rtracking = border(rtracking)
	#amarela
	ytracking = cv2.bitwise_and(yhthresh,cv2.bitwise_and(ysthresh,yvthresh))
	ytracking = border(ytracking)
	#azul
	btracking = cv2.bitwise_and(bhthresh,cv2.bitwise_and(bsthresh,bvthresh))
	btracking = border(btracking)
	
	# Some morpholigical filtering
	#erode = cv2.erode(rtracking, kernel, iterations = 1)

	#vermelha	
	rdilation = cv2.dilate(rtracking,kernel,iterations = 1)
	rclosing = cv2.morphologyEx(rdilation, cv2.MORPH_CLOSE, kernel)
	rclosing = cv2.GaussianBlur(rclosing,(9,9),0)
	#rtracking = border(rtracking)
	ret2, rclosing = cv2.threshold(rclosing,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	#amarela
	ydilation = cv2.dilate(ytracking,kernel,iterations = 1)
	yclosing = cv2.morphologyEx(ydilation, cv2.MORPH_CLOSE, kernel)
	yclosing = cv2.GaussianBlur(yclosing,(9,9),0)
	#ytracking = border(ytracking)
	ret2, yclosing = cv2.threshold(yclosing,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	#azul
	bdilation = cv2.dilate(btracking,kernel,iterations = 1)
	bclosing = cv2.morphologyEx(bdilation, cv2.MORPH_CLOSE, kernel)
	bclosing = cv2.GaussianBlur(bclosing,(9,9),0)
	#btracking = border(btracking)]
	ret2, bclosing = cv2.threshold(bclosing,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

	# Detect circles using HoughCircles -r
	rcircles = cv2.HoughCircles(rclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	verificaR=0;
	if rcircles is not None:
		for i in rcircles[0,:]:
			#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
			

			#get distance for each pixel
			
			#print distance for a center pixel of sphere
			if(i[0] < 600):
				
				rzCM = get_distance_St(depthOriginal[int(round(i[1])) - i[2]][int(round(i[0])) + i[2]])
				#draw_str(frame, (int(round(i[1]+i[2])), int(round(i[0]+i[2]))), '%.2f' % zCM)
				#print i[2]
				rxCM = calcXCM(i[0],rzCM)
				ryCM = calcYCM(i[1],rzCM)
				draw_str(frame, (int(round(i[0]+i[2])), int(round(i[1]+i[2]))), 'x: %.2f y: %.2f z: %.2f' % (rxCM, ryCM, rzCM))
				#print 'x: %f y: %f z: %f' % (xCM, yCM, zCM)
				#print zCM
				#logData.write('rX: %.2f\t rY:%.2f\t rZ: %.2f\t' % (rxCM, ryCM, rzCM))
				#logData.write('%.2f\n' % yCM)
				#countData = countData + 1
				verificaR=1;
			
			# draw a circle around the object in the original image
			cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
			# draw a circle around the object in the depth image
			# todo: verify the diference between 2 frames
			cv2.circle(depth,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(depth,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)

	# Detect circles using HoughCircles -y
	ycircles = cv2.HoughCircles(yclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	verificaY=0;
	if ycircles is not None:
		for i in ycircles[0,:]:
			#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
			

			#get distance for each pixel
			
			#print distance for a center pixel of sphere
			if(i[0] < 600):
				yzCM = get_distance_St(depthOriginal[int(round(i[1])) - i[2]][int(round(i[0])) + i[2]])
				#draw_str(frame, (int(round(i[1]+i[2])), int(round(i[0]+i[2]))), '%.2f' % zCM)
				#print i[2]
				yxCM = calcXCM(i[0],yzCM)
				yyCM = calcYCM(i[1],yzCM)
				draw_str(frame, (int(round(i[0]+i[2])), int(round(i[1]+i[2]))), 'x: %.2f y: %.2f z: %.2f' % (yxCM, yyCM, yzCM))
				#print 'x: %f y: %f z: %f' % (xCM, yCM, zCM)
				#print zCM
				#logData.write('yX: %.2f\t yY:%.2f\t yZ: %.2f\t' % (yxCM, yyCM, yzCM))
				#logData.write('%.2f\n' % yCM)
				#countData = countData + 1
				verificaY=1;
			
			# draw a circle around the object in the original image
			cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
			# draw a circle around the object in the depth image
			# todo: verify the diference between 2 frames
			cv2.circle(depth,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(depth,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)

	# Detect circles using HoughCircles -b
	bcircles = cv2.HoughCircles(bclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	verificaB=0;
	if bcircles is not None:
		for i in bcircles[0,:]:
			#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
		

			#get distance for each pixel
			
			#print distance for a center pixel of sphere
			if(i[0] < 600):
				bzCM = get_distance_St(depthOriginal[int(round(i[1])) - i[2]][int(round(i[0])) + i[2]])
				#draw_str(frame, (int(round(i[1]+i[2])), int(round(i[0]+i[2]))), '%.2f' % zCM)
				#print i[2]
				bxCM = calcXCM(i[0],bzCM)
				byCM = calcYCM(i[1],bzCM)
				draw_str(frame, (int(round(i[0]+i[2])), int(round(i[1]+i[2]))), 'x: %.2f y: %.2f z: %.2f' % (bxCM, byCM, bzCM))
				verificaB=1;	
				#print 'x: %f y: %f z: %f' % (xCM, yCM, zCM)
				#print zCM
				#logData.write('bX: %.2f\t bY:%.2f\t bZ: %.2f\n' % (bxCM, byCM, bzCM))
				#logData.write('%.2f\n' % yCM)
				#countData = countData + 1
			
			
			# draw a circle around the object in the original image
			cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
			# draw a circle around the object in the depth image
			# todo: verify the diference between 2 frames
			cv2.circle(depth,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
			cv2.circle(depth,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
	
	if ((verificaR==1)and(verificaY==1)and(verificaB==1)):
		#Log de dados no TXT
		logData.write('rX: %.2f\trY: %.2f\trZ: %.2f\tyX: %.2f\t yY:%.2f\t yZ: %.2f\tbX: %.2f\tbY: %.2f\t bZ: %.2f\n' % (rxCM, ryCM, rzCM, yxCM, yyCM, yzCM,bxCM, byCM, bzCM))
		print "Tempo: ", elapsedTime
		#formatacao para enviar os dados na serial	
		init='a'
		bin = struct.pack('c',init)
		i=1
		while i==1:
			for x in bin:			  
				ser.write(x)
			i=i+1
		bin = struct.pack('ffffff',rxCM,ryCM,yxCM,yyCM,bxCM,byCM)
		i=1
		while i==1:
			for y in bin:			    
				ser.write(y)
			i=i+1
	

	#draw center coordinates
	cv2.line(frame, (300,240),(340,240),(255,0,0),2) 
	cv2.line(frame, (320,220),(320,260),(255,0,0),2)
  	
	#Show the result  frames
	cv2.imshow('rHueAdj',rhthresh)
	cv2.imshow('rSatAdj',rsthresh)
	cv2.imshow('rValAdj',rvthresh)
        cv2.imshow('yHueAdj',yhthresh)
	cv2.imshow('ySatAdj',ysthresh)
	cv2.imshow('yValAdj',yvthresh)
 	cv2.imshow('bHueAdj',bhthresh)
	cv2.imshow('bSatAdj',bsthresh)
	cv2.imshow('bValAdj',bvthresh)
	cv2.imshow('R-Closing',rclosing)
	cv2.imshow('Y-Closing',yclosing)
	cv2.imshow('B-Closing',bclosing)
	cv2.imshow('Original',frame)
	cv2.imshow('Depth', depth)
	cv2.imshow('Bordas', imgRGB)	
		
	#wait some time
	key_pressed = cv2.waitKey(1)
	endTime = timeit.default_timer()
	elapsedTime = endTime - startTime
	#fim = time.time()
	#print "inicio",init
	#print "fim",fim
logData.close()

