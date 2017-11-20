import random

import numpy as np
import pygame

from constants import Units
from data_structure import Particle, World, SimulatedRobot, RobotSpec, DistanceSensor, FloorSensor
from display import SimulatorWindow

# ---- PARTICLE FILTER PARAMS
N_PARTICLES = 500
POS_SIGMA = 0.05
H_SIGMA = 0.05

# ---- ROBOT BEHAVIOUR PARAMS
WALL_DISTANCE = 0.15

def update_particle_weights(robot, world, pos, h, particle_readings):
    robot_sensor_readings = robot.get_expected_sensor_outputs()
    particle_weights = np.zeros(N_PARTICLES)
    for i in range(N_PARTICLES):
        if Particle.is_position_valid(world, pos[:, i]):
            particle_weights[i] = Particle.get_sensor_reading_probabilities(robot.robot_spec, robot_sensor_readings,
                                                                            particle_readings[i])

    return particle_weights / np.sum(particle_weights)


# ---- ROBOT SPEC
# ------------------------------------------------------------
# Define the robot we will be working with
sensors = [
    ('range', DistanceSensor([0, 0], 0, sigma=0.1)),
    ('right', DistanceSensor([0, 0], np.pi / 2, sigma=0.1)),
    ('left', DistanceSensor([0, 0], -np.pi / 2, sigma=0.1)),
    ('floor', FloorSensor([0, 0]))]
robot_spec = RobotSpec(sensors)
# ------------------------------------------------------------

window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

robot = SimulatedRobot(robot_spec, world, np.array([[0.45], [0.45]]) * Units.METERS_IN_A_FOOT,
                       h=np.array([np.pi / 2]))

scale_mat = np.array([[8 * Units.METERS_IN_A_FOOT, 0], [0, 4 * Units.METERS_IN_A_FOOT]])

# Position of particles in m
start_x = np.random.randint(8, size=(N_PARTICLES,)) + 0.5
start_y = np.random.randint(4, size=(N_PARTICLES,)) + 0.5
starting_positions = np.stack((start_x, start_y), axis=0) * Units.METERS_IN_A_FOOT +  np.random.standard_normal((2, N_PARTICLES)) * POS_SIGMA
#particle_pos = scale_mat @ np.random.rand(2, N_PARTICLES)
particle_pos = starting_positions

# Heading in radians
particle_h = np.random.randint(4, size=N_PARTICLES) * np.pi / 2 + np.random.standard_normal((N_PARTICLES,)) * H_SIGMA
print(particle_pos.shape)

# Initial weights.
particle_readings = [Particle.get_expected_sensor_outputs(robot_spec, world, particle_pos[:, i], particle_h[i]) for i in
                     range(N_PARTICLES)]
particle_weights = update_particle_weights(robot, world, particle_pos, particle_h, particle_readings)

# Draw to screen.
should_quit = False
t = 0
UPDATE_EVERY = 10
update_clock = pygame.time.Clock()
com_pos, com_h, com_uncertainty = np.array([0, 0]), 0, np.array([1, 1, 360])
while not should_quit:
    t += 1
    # Limit fps.
    update_clock.tick(30)
    print("Update took {} ms".format(update_clock.get_time()))

    # ---- DRAW OBJECTS
    # ------------------------------------------------------------
    window.draw()

    for i in range(N_PARTICLES):
        Particle.draw(window, particle_pos[:, i], particle_h[i])
    # Draw the center of mass position and heading
    Particle.draw_com(window, com_pos, com_h, com_uncertainty)

    robot.draw(window)

    # Go ahead and update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()
    # ------------------------------------------------------------

    # Process events.
    mouse_clicked = False
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            should_quit = True
        if event.type == pygame.MOUSEBUTTONUP:
            mouse_clicked = True

    robot_sensor_readings = robot.get_expected_sensor_outputs()
    while robot_sensor_readings.range < WALL_DISTANCE:
        turning_angle = np.pi / 2 if np.random.random() < 0.5 else -np.pi / 2
        robot.move(0, turning_angle + np.random.normal(scale=0.05))
        particle_pos, particle_h = Particle.move(particle_pos, particle_h, 0, turning_angle)
        robot_sensor_readings = robot.get_expected_sensor_outputs()
    # Move forward
    speed = 0.015
    robot.move(speed, 0)
    particle_pos, particle_h = Particle.move(particle_pos, particle_h, speed, 0)

    # Update particle weights.
    particle_readings = [Particle.get_expected_sensor_outputs(robot_spec, world, particle_pos[:, i], particle_h[i]) for
                         i in range(N_PARTICLES)]
    particle_weights = update_particle_weights(robot, world, particle_pos, particle_h, particle_readings)

    # Update center of mass - best estimate
    com_pos = np.mean(particle_pos, axis=1)
    com_h = np.mean(particle_h)
    com_uncertainty = np.std(np.concatenate([particle_pos, np.expand_dims(particle_h, 0)], axis=0), axis=0)

    if not (t % UPDATE_EVERY == 0):
        continue

    # Resample particles
    print('Resampling...')

    particle_samples = np.random.choice(N_PARTICLES, size=(N_PARTICLES,), replace=True, p=particle_weights)
    particle_pos = np.take(particle_pos, particle_samples, axis=1) + np.random.standard_normal(
        (2, N_PARTICLES)) * POS_SIGMA
    particle_h = np.take(particle_h, particle_samples) + np.random.standard_normal((N_PARTICLES,)) * H_SIGMA