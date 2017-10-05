import math

import pygame

from constants import Colours
from data_structure import Particle
from display import draw_triangle


class SimulatedRobot(Particle):
    def __init__(self, world, x, y, h):
        super().__init__(world, x, y, h)

    def draw(self, window):
        x_px = window.m_to_px(self.x)
        y_px = window.m_to_px(self.y)

        # Robot
        draw_triangle(window.screen, x_px, y_px, self.h, r=7, c=Colours.ROBOT_COLOUR, fill=True)

        # Line follower indicator.
        line_reading = self.world.get_line_reading(self.x, self.y)
        if line_reading is not None:
            pygame.draw.circle(window.screen,
                               Colours.BLACK if line_reading else Colours.WHITE,
                               [x_px, y_px], 2)

        # Rangefinder line.
        h_rad = math.radians(self.h)
        dx = math.sin(h_rad)
        dy = -math.cos(h_rad)
        rangefinder_distance = self.world.get_rangefinder_distance(self.x, self.y, self.h)
        pygame.draw.line(window.screen, Colours.BLUE, (window.m_to_px(self.x), window.m_to_px(self.y)),
                         (window.m_to_px(self.x + rangefinder_distance * dx),
                          window.m_to_px(self.y + rangefinder_distance * dy)))
