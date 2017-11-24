import math

import pygame
import numpy as np

from constants import Colours
from data_structure import Particle
from display import Drawable, draw_triangle


class SimulatedRobot(Drawable):
    def __init__(self, robot_spec, world, pos, h):
        self.robot_spec = robot_spec
        self.world = world
        self.pos = pos
        self.h = h

    def get_sensor_outputs(self):
        return Particle.get_expected_sensor_outputs(self.robot_spec, self.world, self.pos, self.h)

    def move(self, d_d, d_h):
        self.pos, self.h = Particle.move(self.pos, self.h, d_d, d_h)

    def draw(self, window):
        pos_px = window.m_to_px(self.pos)

        # Robot
        draw_triangle(window.screen, int(pos_px[0]), int(pos_px[1]), self.h, r=7, c=Colours.ROBOT_COLOUR, fill=True)

        readings = Particle.get_expected_sensor_outputs(self.robot_spec, self.world, self.pos, self.h)

        for reading, sensor in zip(readings, self.robot_spec.sensors):
            sensor.draw(window, self.pos, self.h, reading)

        # # Line follower indicator.
        # line_reading = readings.floor
        # if line_reading is not None:
        #     pygame.draw.circle(window.screen,
        #                        Colours.BLACK if line_reading else Colours.WHITE,
        #                        [x_px, y_px], 2)

        # # Rangefinder line.
        # h_rad = math.radians(self.h)
        # dx = math.sin(h_rad)
        # dy = -math.cos(h_rad)
        
        # rangefinder_distance = readings.range
        # pygame.draw.line(window.screen, Colours.BLUE, (window.m_to_px(self.x), window.m_to_px(self.y)),
        #                  (window.m_to_px(self.x + rangefinder_distance * dx),
        #                   window.m_to_px(self.y + rangefinder_distance * dy)))
