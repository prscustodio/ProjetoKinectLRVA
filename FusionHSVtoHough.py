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
import Tkinter
#teste
#Values for Red ball
rhmin = 169
rhmax = 189	
rvmin = 100
rvmax = 200
rsmin = 100
rsmax = 200

#vectors to use in openCv functions
kernel = np.ones((5,5),np.uint8)
kernel2 = np.ones((20,20),np.uint8)

rcircles=0
mindist=15
minRadius=2

lim =0

bzCM=0
rzCM=0
yzCM=0
bxCM=0
rxCM=0
yxCM=0
byCM=0
ryCM=0
yyCM=0

#Windows to show images
cv2.namedWindow("Original")
cv2.namedWindow("Hough")
#Vermelha
cv2.namedWindow('rHueAdj')

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

const=1
j=1

DadosProfundidade='/home/paulo/ProjetoKinectLRVA/dadosProfundidadeOficial/dadosProfundidade'


while j:
		flagProx=0
	#while const>0:

		VerificaR=0
		
		DadosProfundidade='/home/paulo/ProjetoKinectLRVA/dadosProfundidadeOficial/dadosProfundidade17'
		FrameSemTratar="/home/paulo/ProjetoKinectLRVA/frames/img20.png"
		FrameSemTratarHOUGH="/home/paulo/ProjetoKinectLRVA/frames/img20.png"
		frame = cv2.imread(FrameSemTratar)
		frameHOUGH = cv2.imread(FrameSemTratarHOUGH)
		arq = open(DadosProfundidade, 'r')
 
		

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
	
		# Apply thresholding
		#vermelha
		rhthresh = cv2.inRange(np.array(hue),np.array(rhmn),np.array(rhmx))
		rsthresh = cv2.inRange(np.array(sat),np.array(rsmn),np.array(rsmx))
		rvthresh = cv2.inRange(np.array(val),np.array(rvmn),np.array(rvmx))
	
		# AND h s and v
		#vermelha
		rtracking = cv2.bitwise_and(rhthresh,cv2.bitwise_and(rsthresh,rvthresh))
	
		# Some morpholigical filtering
		#erode = cv2.erode(rtracking, kernel, iterations = 1)

		#vermelha	
		rdilation = cv2.dilate(rtracking,kernel,iterations = 1)
		rclosing = cv2.morphologyEx(rdilation, cv2.MORPH_CLOSE, kernel)
		rclosing = cv2.GaussianBlur(rclosing,(9,9),0)
		#rtracking = border(rtracking)
		ret2, rclosing = cv2.threshold(rclosing,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
	
		rcircles = cv2.HoughCircles(rclosing,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
	
		if rcircles is not None:
			for i in rcircles[0,:]:
				if(i[0] < 640):
					#draw_str(frame,(int(round(i[1]+i[2])), int(round(i[0]+i[2]))), 'x: ' + str(i[0]) + ' y: ' + str(i[1]))
			
					# draw a circle around the object in the original image
					cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5)
					cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),2,(0,0,255),10)
					valR= i[1]*i[0] #position red in the matrix
					#print (valR)
					cont=0
					#contador pra ler ate a linha referente ao pixel da marcacao e armazenar ela
					while cont<valR:	
						cont=cont+1
						depthOriginal = arq.readline()
					try:
						rzCM=float(str(depthOriginal))
					except ValueError:
						rzCM=rzCM
				        rzCM=get_distance_St(rzCM)
					rxCM = calcXCM(i[0],rzCM)
					ryCM = calcYCM(i[1],rzCM)
					draw_str(frame, (int(round(i[0]+i[2])), int(round(i[1]+i[2]))), 'Dados X = x: %.2f y: %.2f z: %.2f' % (rxCM, ryCM, rzCM))

		#hogh img generate
		#frameHOUGH = cv2.medianBlur(frameHOUGH,5)
     		cframeHOUGH = cv2.cvtColor(frameHOUGH,cv2.COLOR_BGR2GRAY)
		
		#find circles use only HOuGH 
		
		cframeHOUGH = cv2.morphologyEx(cframeHOUGH, cv2.MORPH_CLOSE, kernel)
		cframeHOUGH = cv2.GaussianBlur(cframeHOUGH,(9,9),0)
		
					
		circles = cv2.HoughCircles(cframeHOUGH,cv.CV_HOUGH_GRADIENT,2,mindist,param1=60,param2=30,minRadius=15,maxRadius=20)
    		vardifcor=1
    		circles = np.uint16(np.around(circles))
		for k in circles[0,:]:
			

			print k
			print vardifcor	
			print i
			if vardifcor==4:
				vardifcor=1
			draw_str(frameHOUGH, (int(round(k[0]+k[2])), int(round(k[1]+k[2]))),'%.2f' % (vardifcor))
			vardifcor+=1
			if ((i[0]>(k[0]-lim)and i[0]<(k[0]+lim)) and (i[1]>(k[1]-lim)and i[1]<(k[1]+lim))):
				cv2.circle(frameHOUGH,(int(round(k[0])),int(round(k[1]))),int(round(k[2])),(0,255,0),5)
				# draw the outer circle
				cv2.circle(frameHOUGH,(k[0],k[1]),k[2],(0,255,0),2)
				# draw the center of the circle
				cv2.circle(frameHOUGH,(k[0],k[1]),2,(0,0,255),3)	
				lim=0
			
			lim+=1
		cv2.line(frameHOUGH, (20,20),(20,5),(255,0,0),2) 
		cv2.line(frameHOUGH, (10,10),(10,10),(255,0,0),2)
		#draw center coordinates
		cv2.line(frame, (300,240),(340,240),(255,0,0),2) 
		cv2.line(frame, (320,220),(320,260),(255,0,0),2)
		cv2.imshow("Original",frame)
		cv2.imshow("Hough",frameHOUGH)
		cv2.imshow("R",rclosing)
		cv2.imshow("RHue",rhthresh)
		cv2.imshow("RSat",rsthresh)
		cv2.imshow("RValue",rvthresh)
	
			
		#const= input('para atualiazr digite um numero > que 0: ')
		cv2.waitKey(1000)
		











