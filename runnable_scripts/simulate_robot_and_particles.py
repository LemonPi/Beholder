import math
import random
import time

import numpy as np
from numpy.random import choice

from data_structure import Particle
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
world.add_to_screen(window)

robot = SimulatedRobot(world, x=1, y=1, h=0)
robot.add_to_screen(window)

num_particles = 200
particles = [Particle.create_random(world) for _ in range(num_particles)]
for p in particles:
    p.add_to_screen(window)

while true_until_window_closed():
    # Do some moving
    robot_sensor_readings = robot.get_expected_sensor_outputs()
    while robot_sensor_readings.range < 0.05:
        turning_angle = random.randint(1, 359)
        robot.move(0, turning_angle)
        for p in particles:
            p.move(0, turning_angle)
        robot_sensor_readings = robot.get_expected_sensor_outputs()
    robot.move(0.01, 0)
    for p in particles:
        p.move(0.01, 0)

    # Update particle weights.
    particle_weights = np.zeros(len(particles))
    invalid_particle_indices = []
    for i, p in enumerate(particles):
        if p.is_position_valid():
            particle_sensor_readings = p.get_expected_sensor_outputs()
            particle_weights[i] = sum(w_gauss(r_s, p_s, s) for r_s, p_s, s in
                                      zip(robot_sensor_readings, particle_sensor_readings, sensor_reading_sigmas))
        else:
            # Remove invalid particles.
            invalid_particle_indices.append(i)
    particle_weights /= np.sum(particle_weights)

    for i in invalid_particle_indices:
        particles[i].remove_from_screen(window)
        p = int(choice(len(particles), 1, p=particle_weights))
        tries = 0
        candidate_particle = Particle.create_from(particles[p], sigma_pos=0.1, sigma_h=10)
        while not candidate_particle.is_position_valid() and tries < 3:
            candidate_particle = Particle.create_from(particles[p], sigma_pos=0.1, sigma_h=10)
            tries += 1
        else:
            candidate_particle = Particle.create_random(world)
        particles[i] = candidate_particle
        particles[i].h = robot.h
        particles[i].add_to_screen(window)

    # Draw to screen.
    window.draw()
    # Limit fps.
    # time.sleep(0.01)
