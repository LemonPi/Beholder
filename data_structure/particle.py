import math
import random
import numpy as np

import pygame

from constants import Colours
from display import draw_triangle


class Particle(object):
    
    @classmethod
    def string_rep(cls, pos, w):
        return "(%f, %f, w=%f)" % (pos[0], pos[1], w)

    @classmethod
    def create_from(cls, particle, sigma_pos, sigma_h):
        x = random.normalvariate(particle.x, sigma_pos)
        y = random.normalvariate(particle.y, sigma_pos)
        h = random.normalvariate(particle.h, sigma_h)
        return Particle(robot_spec=particle.robot_spec, world=particle.world, x=x, y=y, h=h)

    @classmethod
    def create_random(cls, robot_spec, world):
        while True:
            p = Particle(robot_spec, world, x=random.random() * world.get_width_m(), y=random.random() * world.get_height_m(),
                         h=random.randint(0, 359))
            if p.is_position_valid():
                return p

    @classmethod
    def get_expected_sensor_outputs(cls, robot_spec, world, pos, h):
        """
        Returns an object containing simulated sensor readings based on the particle's pose. (x_1, ... x_n)
        """
        raw_readings = [s.get_sensor_reading(world, pos, h) for s in robot_spec.sensors]

        return robot_spec.SensorReadingTuple(*raw_readings)
    
    @classmethod
    def get_sensor_reading_probabilities(cls, robot_spec, robot_readings, sensor_readings):
        return np.prod([s.get_reading_probability(truth, r) for truth,r,s in zip(robot_readings, sensor_readings, robot_spec.sensors)])

    @classmethod
    def is_position_valid(cls, world, pos):
        return world.is_free(pos[0], pos[1])

    @classmethod
    def move(cls, pos, h, d_d, d_h):
        h_n = h + d_h
        pos_n = pos + np.array([np.sin(h), -np.cos(h)])*d_d

        return pos_n, h_n

    @classmethod
    def draw(cls, window, pos, h):
        x_px = window.m_to_px(pos[0])
        y_px = window.m_to_px(pos[1])

        draw_triangle(window.screen, x_px, y_px, h, r=5, c=Colours.PARTICLE_COLOUR)
        # pygame.draw.circle(window.screen, Colours.GREEN, [x_px, y_px], int(self.w * 15))
        # pygame.draw.circle(window.screen,
        #                    Colours.BLACK if self.world.get_line_reading(self.x, self.y) else Colours.WHITE,
        #                    [x_px, y_px], 2)
