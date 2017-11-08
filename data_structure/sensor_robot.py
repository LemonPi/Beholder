import math

import pygame
import numpy as np

from constants import Colours
from data_structure import Particle
from display import Drawable, draw_triangle
from robot_spec import SensorReadingPacketizer

from multiprocessing import Process, Value, Array

class SensorRobot(Drawable):
    def __init__(self, robot_spec, last_reading, sensor_readings, pos, h):
        self.robot_spec = robot_spec
        self.world = world
        self.pos = pos
        self.h = h

        self.serial = serial
        self.last_reading = last_reading
        self.sensor_readings = sensor_readings

    def get_sensor_outputs(self):
        return self.sensor_readings

    def move(self, d_d, d_h):
        self.pos, self.h = Particle.move(self.pos, self.h, d_d, d_h)

    def set_pos(self, pos, h):
        self.pos = pos
        self.h = h

    def draw(self, window):
        pos_px = window.m_to_px(self.pos)

        # Robot
        draw_triangle(window.screen, int(pos_px[0]), int(pos_px[1]), self.h, r=7, c=Colours.ROBOT_COLOUR, fill=True)

        if self.sensor_readings:
            for reading, sensor in zip(self.sensor_readings, self.robot_spec.sensors):
                sensor.draw(window, self.pos, self.h, reading)
