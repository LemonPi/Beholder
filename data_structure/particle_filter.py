from constants import Units
from data_structure import Particle

import numpy as np

class ParticleFilter(object):
    def __init__(self, num_particles, pos_sigma, h_sigma):
        self.N_PARTICLES = num_particles
        self.POS_SIGMA = pos_sigma
        self.H_SIGMA = h_sigma

        # Position of particles in m
        start_x = np.random.randint(8, size=(self.N_PARTICLES,)) + 0.5
        start_y = np.random.randint(4, size=(self.N_PARTICLES,)) + 0.5
        starting_positions = np.stack((start_x, start_y), axis=0) * Units.METERS_IN_A_FOOT +  np.random.standard_normal((2, self.N_PARTICLES)) * self.POS_SIGMA
        
        # Initial guess for particle positions
        self.particle_pos = starting_positions

        # Initial guess for particle headings [radians]
        self.particle_h = np.random.randint(4, size=self.N_PARTICLES) * np.pi / 2 + np.random.standard_normal((self.N_PARTICLES,)) * self.H_SIGMA

        # Initial guess at particle probabilities
        self.particle_weights = np.zeros(self.N_PARTICLES)

    def update_particle_weights(self, robot_spec, robot_sensor_readings, world):
        particle_readings = [Particle.get_expected_sensor_outputs(robot_spec, world, self.particle_pos[:, i,np.newaxis], self.particle_h[i]) for
                         i in range(self.N_PARTICLES)]

        self.particle_weights = np.zeros(self.N_PARTICLES)
        for i in range(self.N_PARTICLES):
            if Particle.is_position_valid(world, self.particle_pos[:, i]):
                self.particle_weights[i] = Particle.get_sensor_reading_probabilities(robot_spec, robot_sensor_readings,
                                                                                particle_readings[i])

        self.particle_weights =  self.particle_weights / np.sum(self.particle_weights)

    def resample(self):
        particle_samples = np.random.choice(self.N_PARTICLES, size=(self.N_PARTICLES,), replace=True, p=self.particle_weights)
        self.particle_pos = np.take(self.particle_pos, particle_samples, axis=1) + np.random.standard_normal(
            (2, self.N_PARTICLES)) * self.POS_SIGMA
        self.particle_h = np.take(self.particle_h, particle_samples) + np.random.standard_normal((self.N_PARTICLES,)) * self.H_SIGMA

    def move_particles(self, d, dh):
        self.particle_pos, self.particle_h = Particle.move(self.particle_pos, self.particle_h, d, dh)

    def get_pose_estimate(self):
        com_pos = np.mean(self.particle_pos, axis=1)
        com_h = np.mean(self.particle_h)
        com_uncertainty = np.std(np.concatenate([self.particle_pos, np.expand_dims(self.particle_h, 0)], axis=0), axis=0)
        
        return (com_pos, com_h, com_uncertainty)