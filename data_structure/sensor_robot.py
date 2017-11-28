import math

import pygame
import numpy as np

from constants import Colours
from data_structure import Particle
from display import Drawable, draw_triangle

from multiprocessing import Process, Value, Array

class SensorRobot(Drawable):
    def __init__(self, robot_spec):        
        self.robot_spec = robot_spec
        self.pos = None
        self.h = None
        self.sensor_readings = None
        self.localized = False
        self.state = 0
    
    def draw(self, window):
        if(self.localized):
            pos_px = window.m_to_px(self.pos)

            # Robot
            color = (0, 255, 0) if self.state == 5 else Colours.ROBOT_COLOUR 
            draw_triangle(window.screen, int(pos_px[0]), int(pos_px[1]), self.h, r=7, c=color, fill=True)

            readings = self.sensor_readings[:]
            if readings:
                for reading, sensor in zip(readings, self.robot_spec.sensors):
                    sensor.draw(window, self.pos, self.h, reading)
