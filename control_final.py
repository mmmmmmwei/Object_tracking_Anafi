#!/usr/bin/env python

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

class StreamingExample:

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone(
            "10.202.0.1",
            loglevel=3,
        )

    def start(self):
        # Connect the the drone
        pygame.init()
        W, H = 320, 240
        fps=40
        vx=0;
        vy=0;
        vz=0.0;
        vr=0.0;
        screen = pygame.display.set_mode((W, H))
        self.drone.connection()
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

            self.drone.piloting_pcmd(vy, vx, 0, 0, 0.5) # (roll, pitch, yaw, gaz, piloting_time)
            time.sleep(0.2)
            pygame.display.flip()
            #clock.tick(50)
            #pygame.display.set_caption("FPS: %.2f" % clock.get_fps())


            #clock.tick(fps)



if __name__ == "__main__":
    streaming_example = StreamingExample()
    streaming_example.start()

