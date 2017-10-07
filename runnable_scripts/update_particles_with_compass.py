import random
import time

import numpy as np
import pygame
from numpy.random import choice

from constants import Units
from data_structure import Particle
from data_structure import World, SimulatedRobot
from display import SimulatorWindow

sensor_reading_sigmas = (0.1, 1)


def w_gauss(a, b, sigma):
    # This is just a gaussian kernel I pulled out of my hat, to transform
    # values near to robbie's measurement => 1, further away => 0
    error = a - b
    g = np.exp(-(error ** 2 / (2 * sigma ** 2)))
    return g


def update_particle_weights(robot, particles):
    robot_sensor_readings = robot.get_expected_sensor_outputs()
    particle_weights = np.zeros(len(particles))
    for i, p in enumerate(particles):
        if p.is_position_valid():
            particle_sensor_readings = p.get_expected_sensor_outputs()
            particle_weights[i] = np.prod([w_gauss(r_s, p_s, s) for r_s, p_s, s in
                                           zip(robot_sensor_readings, particle_sensor_readings,
                                               sensor_reading_sigmas)])
    particle_weights /= np.sum(particle_weights)
    max_weight = np.max(particle_weights)
    for i, p in enumerate(particles):
        p.w = particle_weights[i] / max_weight


window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

robot = SimulatedRobot(world, x=1.5, y=1, h=45)
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
should_quit = False
t = 0
UPDATE_EVERY = 10
while not should_quit:
    t += 1
    # Limit fps.
    time.sleep(0.1)
    # Draw to screen.
    window.draw()

    # Process events.
    mouse_clicked = False
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            should_quit = True
        if event.type == pygame.MOUSEBUTTONUP:
            mouse_clicked = True

    # Do some moving.
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
    update_particle_weights(robot, particles)

    if not (t % UPDATE_EVERY == 0):
        continue

    # Update particles.
    print('Running update...')
    new_particles = []
    could_not_place = 0
    for i in range(len(particles)):
        particles[i].remove_from_window(window)
        tries = 0
        p = int(choice(len(particles), 1, p=particle_weights))
        candidate_particle = Particle.create_from(particles[p], sigma_pos=0.01, sigma_h=10)
        while not candidate_particle.is_position_valid():
            tries += 1
            p = int(choice(len(particles), 1, p=particle_weights))
            candidate_particle = Particle.create_from(particles[p], sigma_pos=0.01, sigma_h=10)
            if tries >= 3:
                could_not_place += 1
                candidate_particle = Particle.create_random(world)
                break
        candidate_particle.h = robot.h
        candidate_particle.add_to_window(window)
        new_particles.append(candidate_particle)
    print('Could not place {} particles'.format(could_not_place))
    particles = new_particles

    update_particle_weights(robot, particles)
