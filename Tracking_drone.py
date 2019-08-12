
#import opencv library
import numpy as np
import cv2

#greenLower = (80, 160, 80)
#greenUpper = (90, 255, 255)


import olympe
import olympe_deps as od
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
import time

#define color range
greenLower = (70, 160, 80)
greenUpper = (100, 255, 255)

SIMULATION = True # choose wether to use simulated drone or physical one
file_object  = open('drone_log.txt', 'w', encoding='UTF-8') # create a file to store logs

if SIMULATION:
   # connect to simulated drone in SPHINX.
   drone = olympe.Drone("10.202.0.1", logfile=file_object) # create Drone instance
else:
   # connect to physical drone through the Sky Controller
   # you can also connect to drone using Wifi using ip address 192.168.42.1
   drone = olympe.Drone("192.168.53.1", logfile=file_object, mpp=True, drone_type=od.ARSDK_DEVICE_TYPE_ANAFI4K)

# connect to the drone and check if connection was established
response = drone.connection()
if not response.OK: # if connection failed, exit
   print("Connection was not successful")
   exit()

# set piloting source to controller, only when connecting through the Sky Controller
if not SIMULATION:
   drone(setPilotingSource(source="Controller")).wait()

print("TakingOff")
drone(
    TakeOff()
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait()

#start streaming
cam=cv2.VideoCapture(0)

while True:
	ret_value,img=cam.read()

	blurred = cv2.GaussianBlur(img, (11, 11), 0)
	hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

	# Threshold the HSV image to get only certain colors
	mask = cv2.inRange(hsv, greenLower, greenUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	mask = cv2.GaussianBlur(mask, (15, 15), 2, 2)
	     
	contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	#shape = cv2.drawContours(img, contours, -1, (0,255,0), 3)

	vx=0.0;
	vy=0.0;
	vz=0.0;
	vr=0.0;
	marker_y=0.0;
	marker_x=0.0;
	kp=0.002;
	largest=None;

	     
	for cnt in contours:
		approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
		#print (len(approx))
		if len(approx)==4:
		    print ("square")
		    cv2.drawContours(img,[cnt],0,(0,0,255),-1)
		elif len(approx) > 13:
			print ("circle")
			cv2.drawContours(img,[cnt],0,(0,255,255),-1)
			if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
				if cv2.contourArea(cnt)<20000:
					largest = cnt
				
	
	if largest is not None:
		cv2.drawContours(img, largest, -1, (0,255,0), 3)
		M=cv2.moments(largest)
		marker_y=int(M["m01"]/M["m00"])
		marker_x=int(M["m10"]/M["m00"])	
		vx=0.1
		vy=0.0
		vz=kp*(img.shape[0]/2-marker_y)
		vr=kp*(img.shape[1]/2-marker_x)
		cv2.circle(img,(marker_x,marker_y),2,(0,0,255),-1)


	cv2.circle(img,(int(img.shape[1]/2),int(img.shape[0]/2)),2,(0,0,255),-1)	
	  
	cv2.imshow('img',img)

	drone(moveBy(vx, vy, -vz, -vr)).wait(10)
	
	if cv2.waitKey(1) == 27:
		break


cv2.destroyAllWindows()
drone.disconnection()
