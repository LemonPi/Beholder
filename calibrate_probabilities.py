import numpy as np

from constants import Units
from data_structure import Particle
from data_structure import World, SimulatedRobot
from display import SimulatorWindow, block_until_window_closed

sensor_reading_sigmas = (0.1, 1)


def w_gauss(a, b, sigma):
    # This is just a gaussian kernel I pulled out of my hat, to transform
    # values near to robbie's measurement => 1, further away => 0
    error = a - b
    g = np.exp(-(error ** 2 / (2 * sigma ** 2)))
    return g


window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

robot = SimulatedRobot(world, pos=[1, 1], h=0)
robot.add_to_window(window)

particles = []
for x in range(8 * 10):
    for y in range(4 * 10):
        p = Particle(world, x / (8 * Units.METERS_IN_A_FOOT * 10), y / (4 * Units.METERS_IN_A_FOOT * 10), h=robot.h)
        p.add_to_window(window)
        particles.append(p)

# Update particle weights.
robot_sensor_readings = robot.get_expected_sensor_outputs()
particle_weights = np.zeros(len(particles))
invalid_particle_indices = []
for i, p in enumerate(particles):
    if p.is_position_valid():
        particle_sensor_readings = p.get_expected_sensor_outputs()
        particle_weights[i] = np.prod([w_gauss(r_s, p_s, s) for r_s, p_s, s in
                                       zip(robot_sensor_readings, particle_sensor_readings, sensor_reading_sigmas)])
particle_weights /= np.sum(particle_weights)
max_weight = np.max(particle_weights)
for i, p in enumerate(particles):
    p.w = particle_weights[i] / max_weight

# Draw to screen.
window.draw()
block_until_window_closed()
