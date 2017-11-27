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
    def __init__(self, pos):
        # Position relative to the robot origin
        self.pos = pos

    def get_position(self, rpos, heading):
        rot = np.array([[np.cos(heading), -np.sin(heading)], 
                         [np.sin(heading),  np.cos(heading)]])

        return rot @ self.pos + rpos
    
    def get_sensor_reading(self, world, pos, heading):
        raise NotImplementedError

    def get_reading_probability(self, absolute, reading):
        """
        Get likelihood of sensing a reading given what a perfect
        reading should read and what the sensor's error is.
        :param absolute: An ideal sensor reading from current pose
        :param reading: A noisy sensor reading
        :return: Likelihood of reading
        """
        raise NotImplementedError

    def draw(self, window, pos, heading, reading):
        raise NotImplementedError


class DistanceSensor(Sensor):
    def __init__(self, pos, orientation, sigma=0.1):
        super().__init__(pos)
        # Orientation relative to robot
        self.orientation = orientation
        self.sigma = sigma

    def get_sensor_reading(self, world, rpos, heading):
        pos = self.get_position(rpos, float(heading))        
        return world.get_rangefinder_distance(float(pos[0]), float(pos[1]), heading + self.orientation)
    
    def get_reading_probability(self, absolute, reading):
        # Readings which are out of range are considered to be perfectly accurate
        if (absolute >= 0.499):
            return 1
        else:
            return w_gauss(absolute, reading, self.sigma)

    def draw(self, window, rpos, heading, reading):
        pos = self.get_position(rpos, float(heading))

        h = heading + self.orientation
        dx = math.sin(h)
        dy = -math.cos(h)

        pygame.draw.line(window.screen, Colours.BLUE, (window.m_to_px(pos[0]), window.m_to_px(pos[1])),
                         (window.m_to_px(pos[0] + reading * dx),
                          window.m_to_px(pos[1] + reading * dy)))


class FloorSensor(Sensor):
    def __init__(self, pos):
        super().__init__(pos)

    def get_sensor_reading(self, world, pos, heading):
        return world.get_line_reading(pos)
    
    def get_reading_probability(self, absolute, reading):
        return w_gauss(absolute, reading, 1)

    def draw(self, window, pos, heading, reading):
        if reading is not None:
            pygame.draw.circle(window.screen,
                               Colours.BLACK if reading else Colours.WHITE,
                               [window.m_to_px(pos[0]), window.m_to_px(pos[1])], 2)
