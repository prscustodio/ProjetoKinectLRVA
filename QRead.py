import cv2
import zbar
from PIL import Image

cv2.namedWindow("#iothack15")
cap = cv2.VideoCapture(0)

scanner = zbar.ImageScanner()
scanner.parse_config('enable')

# Capture frames from the camera
while True:
    ret, output = cap.read()
    if not ret:
	  continue
    gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY, dstCn=0)
    pil = Image.fromarray(gray)
    width, height = pil.size
    raw = pil.tobytes()
    image = zbar.Image(width, height, 'Y800', raw)
    scanner.scan(image)
	
    for symbol in image:
        print 'decoded', symbol.type, 'symbol', '"%s"' % symbol.data

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
