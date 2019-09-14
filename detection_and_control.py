#!/usr/bin/env python

from pynput import keyboard
from pynput.keyboard import Key, Controller, Listener
import csv
import cv2
import math
import os
import shlex
import subprocess
import tempfile

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
import pygame
import time
import sys

greenLower = (40, 62, 0)
greenUpper = (96, 255, 255)

largest=None;

c = Controller()

class StreamingExample:
    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            "10.202.0.1",
            loglevel=3,
        )

        self.drone.connection()

    def start(self):
        # Connect the the drone
        self.drone.connection()

        print("Takeoff if necessary...")
        self.drone(
            FlyingStateChanged(state="hovering", _policy="check")
            | FlyingStateChanged(state="flying", _policy="check")
            | (
                GPSFixStateChanged(fixed=1, _timeout=10, _policy="check_wait")
                >> (
                    TakeOff(_no_expect=True)
                    & FlyingStateChanged(
                        state="hovering", _timeout=10, _policy="check_wait")
                )
            )
        ).wait()

        self.drone(moveBy(0,-1.15,-0.5,-3.142)>> FlyingStateChanged(state="hovering",_timeout=10)).wait()

        self.drone(moveBy(2.0,0, 0,0)>> FlyingStateChanged(state="hovering",_timeout=10)).wait()


        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb
        )
        # Start video streaming
        self.drone.start_video_streaming()


    def control(self):
        pygame.init()
        W, H = 320, 240
        fps=40
        vx=0;
        vy=0;
        vz=0.0;
        vr=0.0;
        screen = pygame.display.set_mode((W, H))
        #clock = pygame.time.Clock()
        running=True
        while running:
            self.drone.start_piloting()
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    running=False
                elif event.type==pygame.KEYUP:
                    vx=0
                    vy=0
                elif event.type==pygame.KEYDOWN:
                    if event.key==pygame.K_ESCAPE:
                        self.drone.piloting_pcmd(0, 0, 0, 0, 0.0) # (roll, pitch, yaw, gaz, piloting_time)
                        time.sleep(1)
                        running=False
                    elif event.key==pygame.K_p:
                        self.drone(
    TakeOff()
    >> FlyingStateChanged(state="hovering", _timeout=5)
).wait()
                    elif event.key==pygame.K_SPACE:
                        self.drone(
    Landing()
    >> FlyingStateChanged(state="landing", _timeout=5)
).wait()

                    elif event.key==pygame.K_w:
                        vx=20
                    elif event.key==pygame.K_s:
                        vx=-20
                    elif event.key==pygame.K_a:
                        vy=-20
                    elif event.key==pygame.K_d:
                        vy=20

            self.drone.piloting_pcmd(vy, vx, 0, 0, 0.3) # (roll, pitch, yaw, gaz, piloting_time)
            time.sleep(0.2)
            pygame.display.flip()

    def yuv_frame_cb(self, yuv_frame):
        """
        This function will be called by Olympe for each decoded YUV frame.

            :type yuv_frame: olympe.VideoFrame
        """
        # the VideoFrame.info() dictionary contains some useful informations
        # such as the video resolution
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]

        # convert pdraw YUV flag to OpenCV YUV flag
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]

        # yuv_frame.as_ndarray() is a 2D numpy array with the proper "shape"
        # i.e (3 * height / 2, width) because it's a YUV I420 or NV12 frame

        # Use OpenCV to convert the yuv frame to RGB
        img = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

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
        kp=0.001;
        global largest;
     
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
                    if cv2.contourArea(cnt)<1000000:
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

        self.drone(moveBy(vx, vy, -vz, -vr)).wait(10)

        cv2.waitKey(1)


    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        cv2.destroyWindow('img')
        #self.drone.disconnection()
        #self.h264_stats_file.close()


def on_press(key):
    if key.char == 'v':
        streaming_example = StreamingExample()
        streaming_example.control()
    if key.char == 'c':
        streaming_example = StreamingExample()
        streaming_example.stop()
        streaming_example.control()

def on_release(key):
    #print('{0} release'.format(
       # key))
    if key == Key.esc:
        # Stop listener
        return False


if __name__ == "__main__":
    while True:
        streaming_example = StreamingExample()
        streaming_example.start()

        with Listener(
        on_press=on_press,
        on_release=on_release) as listener:
            listener.join()
        #cv2.waitKey(1)

    #streaming_example.fly()
    # Start the video stream
