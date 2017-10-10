import numpy as np
import pygame
import math

from constants import Colours

# ---- UTILS
def w_gauss(a, b, sigma):
    # This is just a gaussian kernel I pulled out of my hat, to transform
    # values near to robbie's measurement => 1, further away => 0
    error = a - b
    g = np.exp(-(error ** 2 / (2 * sigma ** 2)))
    return g

# ---- CLASSES
class Sensor(object):
    def __init__(self, x, y):
        # Position relative to the robot origin
        self.x = x
        self.y = y

    def get_position(self, x, y, heading):
        pass
    
    def get_sensor_reading(self, world, x, y, heading):
        raise NotImplementedError
    
    def get_reading_probability(self, absolute, reading):
        raise NotImplementedError

    def draw(self, window, x, y, heading, reading):
        raise NotImplementedError


class DistanceSensor(Sensor):
    def __init__(self, x, y, orientation, sigma=0.1):
        super().__init__(x,y)
        # Orientation relative to robot
        self.orientation = orientation

        self.sigma = sigma

    def get_sensor_reading(self, world, x, y, heading):
        return world.get_rangefinder_distance(x, y, heading + self.orientation)
    
    def get_reading_probability(self, absolute, reading):
        return w_gauss(absolute, reading, self.sigma)

    def draw(self, window, x, y, heading, reading):
        h_rad = math.radians(heading + self.orientation)
        dx = math.sin(h_rad)
        dy = -math.cos(h_rad)

        pygame.draw.line(window.screen, Colours.BLUE, (window.m_to_px(x), window.m_to_px(y)),
                         (window.m_to_px(x + reading * dx),
                          window.m_to_px(y + reading * dy)))

class FloorSensor(Sensor):
    def __init__(self, x, y):
        super().__init__(x,y)

    def get_sensor_reading(self, world, x, y, heading):
        return world.get_line_reading(x, y)
    
    def get_reading_probability(self, absolute, reading):
        return w_gauss(absolute, reading, 1)

    def draw(self, window, x, y, heading, reading):
        if reading is not None:
            pygame.draw.circle(window.screen,
                               Colours.BLACK if reading else Colours.WHITE,
                               [window.m_to_px(x), window.m_to_px(y)], 2)