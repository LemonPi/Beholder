import random

import numpy as np
import pygame
from numpy.random import choice

from constants import Units
from data_structure import Particle, World, SimulatedRobot, RobotSpec, DistanceSensor, FloorSensor
from display import SimulatorWindow

# ---- RENDERING PARAMS
DRAW_CHANCE = 0.1

# ---- PARTICLE FILTER PARAMS
N_PARTICLES = 1000
POS_SIGMA = 0.02
H_SIGMA = 10


def update_particle_weights(robot, particles, particle_readings):
    robot_sensor_readings = robot.get_expected_sensor_outputs()
    particle_weights = np.zeros(len(particles))
    for i, p in enumerate(particles):
        if p.is_position_valid():
            particle_weights[i] = p.get_sensor_reading_probabilities(robot_sensor_readings, particle_readings[i])

    particle_weights /= np.sum(particle_weights)
    max_weight = np.max(particle_weights)
    for i, p in enumerate(particles):
        p.w = particle_weights[i] / max_weight

    return particle_weights


# ---- ROBOT SPEC
# ------------------------------------------------------------
# Define the robot we will be working with
sensors = [
    ('range', DistanceSensor([0, 0], 0, sigma=0.1)),
    ('right', DistanceSensor([0, 0], 90, sigma=0.1)),
    ('left', DistanceSensor([0, 0], -90, sigma=0.1)),
    ('floor', FloorSensor([0, 0]))]
robot_spec = RobotSpec(sensors)
# ------------------------------------------------------------

window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

robot = SimulatedRobot(robot_spec, world, pos=[1.5, 1], h=45)
robot.add_to_window(window)

# Evenly distribute the particles to start
particles = []
for x in range(8 * 10):
    for y in range(4 * 10):
        p = Particle(robot_spec, world, x / (8 * Units.METERS_IN_A_FOOT * 10), y / (4 * Units.METERS_IN_A_FOOT * 10),
                     h=random.randint(0, 359))

        particles.append(p)
        p.add_to_window(window)

# Initial weights.
particle_readings = [p.get_expected_sensor_outputs() for p in particles]
particle_weights = update_particle_weights(robot, particles, particle_readings)

# Draw to screen.
should_quit = False
t = 0
UPDATE_EVERY = 10
update_clock = pygame.time.Clock()
while not should_quit:
    t += 1
    # Limit fps.
    update_clock.tick(30)
    print("Update took {} ms".format(update_clock.get_time()))
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

    # IMPLEMENT ROBOT BEHAVIOUR
    robot_sensor_readings = robot.get_expected_sensor_outputs()
    robot.move(0, 10)
    for p in particles:
        p.move(0, 10)

    # Update particle weights.
    particle_readings = [p.get_expected_sensor_outputs() for p in particles]
    particle_weights = update_particle_weights(robot, particles, particle_readings)

    if not (t % UPDATE_EVERY == 0):
        continue

    # Resample particles
    print('Resampling...')
    new_particles = []
    could_not_place = 0
    for i in range(len(particles)):
        particles[i].remove_from_window(window)
        tries = 0
        p = int(choice(len(particles), 1, p=particle_weights))
        candidate_particle = Particle.create_from(particles[p], sigma_pos=POS_SIGMA, sigma_h=H_SIGMA)
        while not candidate_particle.is_position_valid():
            tries += 1
            p = int(choice(len(particles), 1, p=particle_weights))
            candidate_particle = Particle.create_from(particles[p], sigma_pos=POS_SIGMA, sigma_h=H_SIGMA)
            if tries >= 3:
                could_not_place += 1
                candidate_particle = Particle.create_random(robot_spec, world)
                break

        # Uncomment line to enable compass.
        # candidate_particle.h = robot.h
        candidate_particle.add_to_window(window)
        new_particles.append(candidate_particle)
    print('Could not place {} particles'.format(could_not_place))
    particles = new_particles
