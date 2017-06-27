import cv2
import zbar
from PIL import Image

cv2.namedWindow("#iothack15")
cap = cv2.VideoCapture(0)

#create a reader
scanner = zbar.ImageScanner()

#configure the reader
scanner.parse_config('enable')

while True:

	    #capture webcam frame
	    ret, output = cap.read()
	    if not ret:
		  continue
	    
	    #obtain image data
	    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY, dstCn=0)
	    pil = Image.fromarray(gray)
	    width, height = pil.size
	    raw = pil.tobytes()
	    
	    #wrap image data
	    image = zbar.Image(width, height, 'Y800', raw)
	    
	    #scan the image for barcodes 
	    scanner.scan(image)		

	    for symbol in image:
		loc = symbol.location
	        x = (loc[0][0]+loc[2][0])/2
		y = (loc[0][1]+loc[2][1])/2
		print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data,'x:', '"%f"' % x,'y:', '"%f"' % y
		
		# draw a circle around the object in the original image
		cv2.circle(output,(x,y),1,(0,255,0),5)
		cv2.circle(output,(x,y),5,(0,0,255),10)

	    cv2.imshow("#iothack15", output)

	    # clear stream for next frame
	    #rawCapture.truncate(0)

	    # Wait for the magic key
	    keypress = cv2.waitKey(1) & 0xFF
	    if keypress == ord('q'):
	    	break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows() 
