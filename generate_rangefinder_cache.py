import math
import pickle

from constants import Units
from data_structure import World, RangefinderCache

world = World()

bounds = (8 * Units.METERS_IN_A_FOOT, 4 * Units.METERS_IN_A_FOOT, 2 * math.pi)
precisions = (0.005, 0.005, 0.02)

range_cache = RangefinderCache.generate(world, precisions, bounds)

with open('parallel_range_finder_cache.pkl', 'wb') as f:
    pickle.dump(range_cache, f)
print("Done generating rangefinder data")