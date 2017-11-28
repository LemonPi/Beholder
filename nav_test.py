import numpy as np

from data_structure import Navigator, World
from constants import Units

world = World()
nav = Navigator(world.maze)

com_pos = np.array([3.5, 0.5]) * Units.METERS_IN_A_FOOT
nav.update(com_pos, np.array([7.5,3.5]) * Units.METERS_IN_A_FOOT)

print(nav.path)