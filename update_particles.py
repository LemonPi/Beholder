import numpy as np
import pickle
import pygame
import random

from constants import Units
from data_structure import Particle, World, SimulatedRobot, RobotSpec, DistanceSensor, FloorSensor, ParticleFilter
from display import SimulatorWindow

# ---- PARTICLE FILTER PARAMS
N_PARTICLES = 500
POS_SIGMA = 0.03
H_SIGMA = 0.02

# ---- ROBOT BEHAVIOUR PARAMS
WALL_DISTANCE = 0.15
ROBOT_SPEED = 0.5 * Units.METERS_IN_A_FOOT

# ---- ROBOT SPEC
# ------------------------------------------------------------
# Define the robot we will be working with
sensors = [
    ('range', DistanceSensor(np.array([[0], [0.01]]), 0, sigma=0.1)),
    ('right', DistanceSensor(np.array([[0.01], [0]]), np.pi / 2, sigma=0.1)),
    ('left', DistanceSensor(np.array([[-0.01], [0]]), -np.pi / 2, sigma=0.1)),
    #('floor', FloorSensor(np.array([[0], [0]])))
    ]
robot_spec = RobotSpec(sensors)
# ------------------------------------------------------------

window = SimulatorWindow(text='Robot simulation')

# Load the rangefinder cache
with open('range_finder_cache.pkl', 'rb') as f:
    rangefinder_cache = pickle.load(f)
print("Loaded rangefinder cache")

world = World(rangefinder_cache=rangefinder_cache)
world.add_to_window(window)

robot = SimulatedRobot(robot_spec, world, np.array([[0.45], [0.45]]) * Units.METERS_IN_A_FOOT,
                       h=np.array([np.pi / 2]))

# Create teh particle filter
pf = ParticleFilter(N_PARTICLES, POS_SIGMA, H_SIGMA)
pf.update_particle_weights(robot.robot_spec, robot.get_sensor_outputs(), world)

# Draw to screen.
should_quit = False
t = 0
UPDATE_EVERY = 5
update_clock = pygame.time.Clock()
com_pos, com_h, com_uncertainty = np.array([0, 0]), 0, np.array([1, 1, 360])
while not should_quit:
    t += 1
    # Limit fps.
    update_clock.tick(10)
    td = update_clock.get_time()
    print("Update took {} ms".format(update_clock.get_time()))

    # ---- DRAW OBJECTS
    # ------------------------------------------------------------
    window.draw()

    for i in range(N_PARTICLES):
        Particle.draw(window, pf.particle_pos[:, i], pf.particle_h[i])
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

    # Simulated robot behaviour
    robot_sensor_readings = robot.get_sensor_outputs()
    while robot_sensor_readings.range < WALL_DISTANCE:
        turning_angle = np.pi / 2 if np.random.random() < 0.5 else -np.pi / 2
        robot.move(0, turning_angle + np.random.normal(scale=0.05))
        pf.move_particles(0, turning_angle)
        robot_sensor_readings = robot.get_sensor_outputs()
    
    # Move forward
    dist = ROBOT_SPEED * td / 1000
    robot.move(dist, 0)
    pf.move_particles(dist, 0)

    # Update particle weights.
    pf.update_particle_weights(robot.robot_spec, robot.get_sensor_outputs(), world)

    # Update center of mass - best estimate
    (com_pos, com_h, com_uncertainty) = pf.get_pose_estimate()

    if not (t % UPDATE_EVERY == 0):
        continue

    # Resample particles
    print('Resampling...')
    pf.resample()
