#!/usr/bin/env python

import csv
import cv2
import math
import os
import shlex
import subprocess
import tempfile
import numpy
import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.Piloting import moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged
from olympe.messages.ardrone3.PilotingSettings import MaxTilt
from olympe.messages.ardrone3.GPSSettingsState import GPSFixStateChanged
import pygame
import time
import sys
import math
import apriltag
import collections

count=0;

class StreamingExample:

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            "10.202.1.1",
            loglevel=3,
        )


    def start(self):
        self.drone.connection()
        self.drone.start_piloting()
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

        self.drone(moveBy(3.0,0, 0,0)>> FlyingStateChanged(state="hovering",_timeout=10)).wait()

        # You can record the video stream from the drone if you plan to do some
        # post processing.
            # Here, we don't record the (huge) raw YUV video stream
            # raw_data_file=os.path.join(self.tempd,'raw_data.bin'),
            # raw_meta_file=os.path.join(self.tempd,'raw_metadata.json'),
        # Setup your callback functions to do some live video processing
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
        )
        # Start video streaming
        self.drone.start_video_streaming()

        while True:
            cv2.waitKey(1)

    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnection()
        self.h264_stats_file.close()

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
        camera_params = (871.3900560920833, 874.042504288542, 310.2749439858911, 229.99099528051528)
        tag_size = 7
        detector = apriltag.Detector()
        # Use OpenCV to convert the yuv frame to RGB
        img = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)

        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY) # convert to grayscale

    # detect the apriltags in the image
        detections, dimg = detector.detect(gray, return_image=True)

        num_detections = len(detections)

    #test1=enumerate(detections)
   # print(list(enumerate(test1,1)).tostring())

    #print('Detected {} tags.\n'.format(num_detections))

    # overlay on the apriltag
        overlay = img // 2 + dimg[:, :, None] // 2

        vx=0.0;
        vy=0.0;
        vz=0.0;
        vr=0.0;
        vxx=0.0;
        vyy=0.0;
        vzz=0.0;
        marker_y=0.0;
        marker_x=0.0;
        kp=0.001;
        global count;

    # loop through all detected apriltags and print their info,
    # get their pose if camera_params is available
        for i, detection in enumerate(detections):
        #print('Detection {} of {}:'.format(i + 1, num_detections))
        #print()
            print(detection.tostring(indent=2))

            marker_x=detection.center[0]
            marker_y=detection.center[1]

            vx=0.3
            vy=0.0
            vz=kp*(img.shape[0]/2-marker_y)
            vr=kp*(img.shape[1]/2-marker_x)
            cv2.circle(overlay,(int(marker_x),int(marker_y)),2,(0,0,255),-1)

            if camera_params is not None and detection.tag_id is count:
                pose, e0, e1 = detector.detection_pose(detection,
                                                   camera_params,
                                                   tag_size)
                #print(detection.tostring(
                    #collections.OrderedDict([('Pose',pose),
                                             #('InitError', e0),
                                             #('FinalError', e1)]),
                                             #indent=2))
                draw_pose(overlay,
                      camera_params,
                      tag_size,
                      pose)
                print()
                print()

                r33=pose[2, 3]
                #print(r33)

                r11=pose[0, 0]
                r21=pose[1, 0]
                r31=pose[2, 0]
                r32=pose[2, 1]
                r33=pose[2, 2]
                yaw=math.atan(r21/r11)
                pitch=math.atan(-r31/math.sqrt(math.pow(r32,2)+math.pow(r33,2)))
                roll=math.atan(r32/r33)

                vxx=int(roll*100)
                vyy=int(pitch*100)
                vzz=int(yaw*100)
                print(vxx,vyy,vzz)


                distance=pose[2, 3]
                print(distance)
                if (distance > 50):
                    self.drone(moveBy(vx, vy, -vz, -vr)).wait(10)
                else:
                    self.drone.piloting_pcmd(vxx, vyy, vzz, 0, 0.5) # (roll, pitch, yaw, gaz, piloting_time)
                    time.sleep(5)
            print()

        cv2.circle(overlay,(int(img.shape[1]/2),int(img.shape[0]/2)),2,(0,0,255),-1)	
  
        cv2.imshow('img',overlay)

        cv2.waitKey(1)

def draw_pose(overlay, camera_params, tag_size, pose, z_sign=1):

    opoints = numpy.array([
        -1, -1, 0,
         1, -1, 0,
         1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,
         1, -1, -2*z_sign,
         1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5*tag_size

    edges = numpy.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6,
        6, 7,
        7, 4
    ]).reshape(-1, 2)

    fx, fy, cx, cy = camera_params

    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    rvec, _ = cv2.Rodrigues(pose[:3,:3])
    tvec = pose[:3, 3]

    dcoeffs = numpy.zeros(5)

    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    ipoints = numpy.round(ipoints).astype(int)

    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


# camera params fx, fy, cx, cy (optional to get apriltag pose)
camera_params = (871.3900560920833, 874.042504288542, 310.2749439858911, 229.99099528051528)
tag_size = 20
if __name__ == "__main__":
    streaming_example = StreamingExample()
    streaming_example.start()

