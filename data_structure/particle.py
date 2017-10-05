import math

import pygame

from constants import Colours
from display import Drawable, draw_triangle


class Particle(Drawable):
    def __init__(self, world, x, y, h, w=1, noisy=False):
        super().__init__()
        self.world = world
        if noisy:
            pass
            # x, y, heading = add_some_noise(x, y, heading)

        self.x = x
        self.y = y
        self.h = h
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place()) for _ in range(0, count)]

    def get_expected_sensor_outputs(self):
        """
        Returns a tuple of simulated sensor readings based on the particle's pose. (x_1, ... x_n)
        """
        return (0,)

    def _read_ultrasonic(self):
        pass

    def _read_ir(self):
        pass

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
        self.h += d_h
        h_rad = math.radians(self.h)

        dx = math.sin(h_rad)
        dy = -math.cos(h_rad)
        self.x += d_d * dx
        self.y += d_d * dy

    def draw(self, window):
        x_px = window.m_to_px(self.x)
        y_px = window.m_to_px(self.y)

        draw_triangle(window.screen, x_px, y_px, self.h, r=5, c=Colours.PARTICLE_COLOUR)
        pygame.draw.circle(window.screen, Colours.GREEN if self.is_position_valid() else Colours.RED, [x_px, y_px], 2)
