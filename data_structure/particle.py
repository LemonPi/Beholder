import math
import random
import numpy as np

import pygame

from constants import Colours
from display import Drawable, draw_triangle


class Particle(Drawable):
    def __init__(self, robot_spec, world, x, y, h):
        super().__init__()
        self.robot_spec = robot_spec

        self.world = world

        self.x = x
        self.y = y
        self.h = h
        self.w = 0

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

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

    def get_expected_sensor_outputs(self):
        """
        Returns an object containing simulated sensor readings based on the particle's pose. (x_1, ... x_n)
        """
        raw_readings = [s.get_sensor_reading(self.world, self.x, self.y, self.h) for s in self.robot_spec.sensors]

        # rangefinder_distance = self.world.get_rangefinder_distance(self.x, self.y, self.h)
        # line_reading = self.world.get_line_reading(self.x, self.y)
        return self.robot_spec.SensorReadingTuple(*raw_readings)
    
    def get_sensor_reading_probabilities(self, robot_readings, sensor_readings):
        return np.prod([s.get_reading_probability(truth, r) for truth,r,s in zip(robot_readings, sensor_readings, self.robot_spec.sensors)])

    def update_weight(self, sensors_robot):
        pass
        # sensors_p, valid_position = p.get_expected_sensor_outputs(world)
        # if (valid_position):
        #     p.w = some_function(sensors_p, sensors_robot)
        # else:
        #     p.w = 0

    def is_position_valid(self):
        return self.world.is_free(self.x, self.y)

    def move(self, d_d, d_h):
        self.h = (self.h + d_h) % 360
        h_rad = math.radians(self.h)

        dx = math.sin(h_rad)
        dy = -math.cos(h_rad)
        self.x += d_d * dx
        self.y += d_d * dy

    def draw(self, window):
        x_px = window.m_to_px(self.x)
        y_px = window.m_to_px(self.y)

        draw_triangle(window.screen, x_px, y_px, self.h, r=5, c=Colours.PARTICLE_COLOUR)
        # pygame.draw.circle(window.screen, Colours.GREEN, [x_px, y_px], int(self.w * 15))
        # pygame.draw.circle(window.screen,
        #                    Colours.BLACK if self.world.get_line_reading(self.x, self.y) else Colours.WHITE,
        #                    [x_px, y_px], 2)
