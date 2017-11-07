import random
import time

import multiprocessing as mpc
import numpy as np
import pygame
import serial

from data_structure import World, SimulatedRobot, RobotSpec, FloorSensor, DistanceSensor, SensorReadingPacketizer
from display import SimulatorWindow, true_until_window_closed


def serial_reader(port, baud, spec, last_reading, sensor_readings):
    packet_parser = SensorReadingPacketizer(spec)

    ser = serial.Serial(port, baud, timeout=None)

    while(True):
        buffer = ser.read(size=packet_parser.NUM_BYTES)
        last_reading.value, readings = packet_parser.from_packet(buffer)
        sensor_readings[:] = readings

        print(last_reading.value, readings)

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

last_reading = mpc.Value('L', 0)
sensor_reading = mpc.Array('f', range(len(robot_spec.sensors)))

pacman = SensorReadingPacketizer(robot_spec)

# Create a sample sensor reading
# readings = robot_spec.SensorReadingTuple(1.0, 2.0, 3.0, 4.0)
# packet = pacman.to_packet(readings)
# print(packet)
# print(pacman.from_packet(packet))

p = mpc.Process(target=serial_reader, args=('/dev/ttyACM0', 9600, robot_spec, last_reading, sensor_reading))
p.start()

# while(True):
#     print(last_reading.value, sensor_reading[:])
#     time.sleep(1)

# window = SimulatorWindow(text='Robot simulation')

# world = World()
# world.add_to_window(window)

# robot = SimulatedRobot(robot_spec, world, np.array([[0.2],[0.2]]), h=np.array([0]))
# robot.add_to_window(window)

# update_clock = pygame.time.Clock()
# while true_until_window_closed():
#     window.draw()
#     # Go ahead and update the screen with what we've drawn.
#     # This MUST happen after all the other drawing commands.
#     pygame.display.flip()

#     update_clock.tick(4)
#     print("Update took {} ms".format(update_clock.get_time()))
#     while robot.get_expected_sensor_outputs().range < 0.05:
#         robot.move(0, random.random()*np.pi*2)
#     robot.move(0.01, 0)