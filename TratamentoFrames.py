#import required libraries
import numpy as np # numpy for array manipulation
import cv2	#OpenCv
import cv2.cv as cv #Open Cv
import math
import serial 
import struct 
import time
import freenect # Kinect free library
import timeit

#Values for Red ball
rhmin = 169
rhmax = 189	
rvmin = 100
rvmax = 200
rsmin = 100
rsmax = 200

#Values for Yellow ball
yhmin = 18
yhmax = 38
yvmin = 216
yvmax = 242
ysmin = 102
ysmax = 255

#Values for blue ball
bhmin = 106
bhmax = 124
bvmin = 172
bvmax = 250
bsmin = 61
bsmax = 111

#vectors to use in openCv functions
kernel = np.ones((5,5),np.uint8)
kernel2 = np.ones((20,20),np.uint8)

rcircles=0
mindist=15
minRadius=2

#Windows to show images
cv2.namedWindow("Original")
#Vermelha
cv2.namedWindow('rHueAdj')
cv2.namedWindow('yHueAdj')
cv2.namedWindow('bHueAdj')

#ser = serial.Serial('/dev/ttyUSB0',57600)

fourcc =  cv2.cv.CV_FOURCC(*'XVID')
video = cv2.VideoWriter("ImagemKinctRGB.avi", fourcc, 10, (640,480), 1)
video2 = cv2.VideoWriter("ImagemKinctDEPTH.avi", fourcc, 30, (320,240), 1)

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

#function to detect circles
def armazenamento(frame):
	video.write(frame)
	np.savetxt('DadosProfundidade', depthOriginal, delimiter='\n- ') 

#create track bars to adjust H, S and V of images

#vermelha
cv2.createTrackbar('hmin', 'rHueAdj',0,255,nothing)
cv2.createTrackbar('hmax', 'rHueAdj',0,255,nothing)
cv2.createTrackbar('smin', 'rHueAdj',0,255,nothing)
cv2.createTrackbar('smax', 'rHueAdj',255,255,nothing)
cv2.createTrackbar('vmin', 'rHueAdj',0,255,nothing)
cv2.createTrackbar('vmax', 'rHueAdj',0,255,nothing)
cv2.setTrackbarPos('hmin', 'rHueAdj', rhmin)
cv2.setTrackbarPos('hmax', 'rHueAdj', rhmax)
cv2.setTrackbarPos('smin', 'rHueAdj', rsmin)
cv2.setTrackbarPos('smax', 'rHueAdj', rsmax)
cv2.setTrackbarPos('vmin', 'rHueAdj', rvmin)
cv2.setTrackbarPos('vmax', 'rHueAdj', rvmax)

#amarela
cv2.createTrackbar('hmin', 'yHueAdj',0,255,nothing)
cv2.createTrackbar('hmax', 'yHueAdj',0,255,nothing)
cv2.createTrackbar('smin', 'yHueAdj',0,255,nothing)
cv2.createTrackbar('smax', 'yHueAdj',255,255,nothing)
cv2.createTrackbar('vmin', 'yHueAdj',0,255,nothing)
cv2.createTrackbar('vmax', 'yHueAdj',0,255,nothing)
cv2.setTrackbarPos('hmin', 'yHueAdj', yhmin)
cv2.setTrackbarPos('hmax', 'yHueAdj', yhmax)
cv2.setTrackbarPos('smin', 'yHueAdj', ysmin)
cv2.setTrackbarPos('smax', 'yHueAdj', ysmax)
cv2.setTrackbarPos('vmin', 'yHueAdj', yvmin)
cv2.setTrackbarPos('vmax', 'yHueAdj', yvmax)

#azul
cv2.createTrackbar('hmin', 'bHueAdj',0,255,nothing)
cv2.createTrackbar('hmax', 'bHueAdj',0,255,nothing)
cv2.createTrackbar('smin', 'bHueAdj',0,255,nothing)
cv2.createTrackbar('smax', 'bHueAdj',255,255,nothing)
cv2.createTrackbar('vmin', 'bHueAdj',0,255,nothing)
cv2.createTrackbar('vmax', 'bHueAdj',0,255,nothing)
cv2.setTrackbarPos('hmin', 'bHueAdj', bhmin)
cv2.setTrackbarPos('hmax', 'bHueAdj', bhmax)
cv2.setTrackbarPos('smin', 'bHueAdj', bsmin)
cv2.setTrackbarPos('smax', 'bHueAdj', bsmax)
cv2.setTrackbarPos('vmin', 'bHueAdj', bvmin)
cv2.setTrackbarPos('vmax', 'bHueAdj', bvmax)
const=1
j=1
while j:
	
	while const>0:

		frame = cv2.imread("/home/paulo/ProjetoKinectLRVA/frames/img1.png")

		arq = open('/home/paulo/ProjetoKinectLRVA/dadosProfundidadeOficial/dadosProfundidade1', 'r')

		# get 11 bit depth value from kinect	
	
		hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV) #convert RGB image to HSV domain

		hue,sat,val = cv2.split(hsv)

		# get values min and max from trackBars 
		#vermelha
		rhmn = cv2.getTrackbarPos('hmin','rHueAdj') #144#
		rhmx = cv2.getTrackbarPos('hmax','rHueAdj') #180#
		rsmn = cv2.getTrackbarPos('smin','rHueAdj') #196#
		rsmx = cv2.getTrackbarPos('smax','rHueAdj') #255#
		rvmn = cv2.getTrackbarPos('vmin','rHueAdj') #131#
		rvmx = cv2.getTrackbarPos('vmax','rHueAdj') #191#
		#amarela
		yhmn = cv2.getTrackbarPos('hmin','yHueAdj') #144#
		yhmx = cv2.getTrackbarPos('hmax','yHueAdj') #180#
		ysmn = cv2.getTrackbarPos('smin','yHueAdj') #196#
		ysmx = cv2.getTrackbarPos('smax','yHueAdj') #255#
		yvmn = cv2.getTrackbarPos('vmin','yHueAdj') #131#
		yvmx = cv2.getTrackbarPos('vmax','yHueAdj') #191#
		#azul
		bhmn = cv2.getTrackbarPos('hmin','bHueAdj') #144#
		bhmx = cv2.getTrackbarPos('hmax','bHueAdj') #180#
		bsmn = cv2.getTrackbarPos('smin','bHueAdj') #196#
		bsmx = cv2.getTrackbarPos('smax','bHueAdj') #255#
		bvmn = cv2.getTrackbarPos('vmin','bHueAdj') #131#
		bvmx = cv2.getTrackbarPos('vmax','bHueAdj') #191#

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
		#amarela
		ytracking = cv2.bitwise_and(yhthresh,cv2.bitwise_and(ysthresh,yvthresh))
		#azul
		btracking = cv2.bitwise_and(bhthresh,cv2.bitwise_and(bsthresh,bvthresh))
	
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
		#rtracking = border(rtracking)
		ret2, yclosing = cv2.threshold(yclosing,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

		#azul	
		bdilation = cv2.dilate(btracking,kernel,iterations = 1)
		bclosing = cv2.morphologyEx(bdilation, cv2.MORPH_CLOSE, kernel)
		bclosing = cv2.GaussianBlur(bclosing,(9,9),0)
		#rtracking = border(rtracking)
		ret2, bclosing = cv2.threshold(bclosing,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	
		rcircles = cv2.HoughCircles(rclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	
		if rcircles is not None:
			for i in rcircles[0,:]:
				#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
			
				# draw a circle around the object in the original image
				cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
				cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
				val= i[1]*i[0] #position red in the matrix
				#print (val)
				cont=0
				#contador pra ler ate a linha referente ao pixel da marcacao e armazenar ela
				while cont<val:	
					cont=cont+1
					depthOriginal = arq.readline()
					valor=float(str(depthOriginal))
					valor=get_distance_St(valor)
				print (valor)
				bxCM = calcXCM(i[0],valor)
				byCM = calcYCM(i[1],valor)
				draw_str(frame, (int(round(i[0]+i[2])), int(round(i[1]+i[2]))), 'x: %.2f y: %.2f z: %.2f' % (bxCM, byCM, valor))

		ycircles = cv2.HoughCircles(yclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	
		if ycircles is not None:
			for i in ycircles[0,:]:
				#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
			
				# draw a circle around the object in the original image
				cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
				cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
				a =i[1]			
				b=i[0]
				#print (a)
				#print (b)
				val= a*b #position yelow in the matrix
				#print (val)
				cont=0
				#contador pra ler ate a linha referente ao pixel da marcacao e armazenar ela
				while cont<val:	
					cont=cont+1
					depthOriginal = arq.readline()
					valor=float(str(depthOriginal))
				
				#print ('valor: ',depthOriginal)
				valor=get_distance_St(valor)
				print(valor)

		bcircles = cv2.HoughCircles(bclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	
		if bcircles is not None:
			for i in bcircles[0,:]:
				#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
			
				# draw a circle around the object in the original image
				cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
				cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
				val= i[1]*i[0]#position blur in the matrix
				#print (val)
				cont=0
				#contador pra ler ate a linha referente ao pixel da marcacao e armazenar ela
				while cont<val:	
					cont=cont+1
					depthOriginal = arq.readline()
					valor=float(str(depthOriginal))
				print (valor)
		
		cv2.imshow("Original",frame)
		cv2.imshow("R",rclosing)
		cv2.imshow("Hue",rhthresh)
		cv2.imshow("Sat",rsthresh)
		cv2.imshow("Value",rvthresh)
		cv2.imshow("Y",yclosing)
		cv2.imshow("YHue",yhthresh)
		cv2.imshow("YSat",ysthresh)
		cv2.imshow("YValue",yvthresh)
		cv2.imshow("B",bclosing)
		cv2.imshow("BHue",bhthresh)
		cv2.imshow("BSat",bsthresh)
		cv2.imshow("BValue",bvthresh)
		
		#const= input('para atualiazr digite um numero > que 0: ')
		cv2.waitKey(1000)
		











