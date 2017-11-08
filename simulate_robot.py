import random
import time

from data_structure import World, SimulatedRobot, RobotSpec, FloorSensor, DistanceSensor
from display import SimulatorWindow, true_until_window_closed
import numpy as np
import pygame

# ---- ROBOT SPEC
# ------------------------------------------------------------
# Define the robot we will be working with
sensors = [
    ('range', DistanceSensor([0, 0], 0, sigma=0.1)),
    ('right', DistanceSensor([0, 0], np.pi/2, sigma=0.1)),
    ('left', DistanceSensor([0, 0], -np.pi/2, sigma=0.1)),
    ('floor', FloorSensor([0, 0]))]
robot_spec = RobotSpec(sensors)
# ------------------------------------------------------------

window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

robot = SimulatedRobot(robot_spec, world, np.array([[0.2],[0.2]]), h=np.array([0]))
robot.add_to_window(window)

update_clock = pygame.time.Clock()
while true_until_window_closed():
    window.draw()
    # Go ahead and update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()

    update_clock.tick(4)
    print("Update took {} ms".format(update_clock.get_time()))
    while robot.get_expected_sensor_outputs().range < 0.05:
        robot.move(0, random.random()*np.pi*2)
    robot.move(0.01, 0)
