import math
import random
import time

from data_structure import World, SimulatedRobot
from display import SimulatorWindow, true_until_window_closed

sensor_reading_sigmas = (0.1, 0.01)


def w_gauss(a, b, sigma):
    # This is just a gaussian kernel I pulled out of my hat, to transform
    # values near to robbie's measurement => 1, further away => 0
    error = a - b
    g = math.e ** -(error ** 2 / (2 * sigma ** 2))
    return g


window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

robot = SimulatedRobot(world, x=1, y=1, h=0)
robot.add_to_window(window)

while true_until_window_closed():
    # Do some moving
    robot_sensor_readings = robot.get_expected_sensor_outputs()
    while robot_sensor_readings.range < 0.05:
        turning_angle = random.randint(1, 359)
        robot.move(0, turning_angle)
        robot_sensor_readings = robot.get_expected_sensor_outputs()
    robot.move(0.01, 0)

    # Draw to screen.
    window.draw()
    # Limit fps.
    time.sleep(0.01)
