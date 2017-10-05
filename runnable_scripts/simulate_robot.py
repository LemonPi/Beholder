import time

from data_structure import World, SimulatedRobot
from display import SimulatorWindow, true_until_window_closed

window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_screen(window)

robot = SimulatedRobot(world, x=1, y=1, h=0)
robot.add_to_screen(window)

while true_until_window_closed():
    window.draw()
    time.sleep(0.1)
    robot.move(0.01, 1)
