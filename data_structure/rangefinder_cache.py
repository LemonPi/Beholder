import numpy as np
import time

class RangefinderCache(object):
    def __init__(self, data_cache, precisions, bounds):
        self.data_cache = data_cache
        self.precisions = precisions
        self.bounds = bounds

    @classmethod
    def generate(cls, world, precisions, bounds):
        NUM_X_BINS = round((bounds[0]) / precisions[0]) + 1
        NUM_Y_BINS = round((bounds[1]) / precisions[1]) + 1
        NUM_H_BINS = round((bounds[2]) / precisions[2]) + 1

        dist_lookup = np.zeros((NUM_X_BINS, NUM_Y_BINS, NUM_H_BINS))

        start_time = time.time()
        for x_bin in range(NUM_X_BINS):
            print("{:.2f}% Complete - {:.2f}s ellapsed".format(x_bin / NUM_X_BINS * 100, time.time() - start_time))
            for y_bin in range(NUM_Y_BINS):
                for h_bin in range(NUM_H_BINS):
                    x = x_bin * precisions[0]
                    y = y_bin * precisions[1]
                    h = h_bin * precisions[2]
                    dist_lookup[x_bin, y_bin, h_bin] = world.get_rangefinder_distance(x,y,h)
        
        return RangefinderCache(dist_lookup, precisions, bounds)

    def get(self, x, y, h):
        xx = int(round(x / self.precisions[0]))
        yy = int(round(y / self.precisions[1]))
        hh = int(round((float(h) % (np.pi * 2)) / self.precisions[2]))

        return self.data_cache[xx, yy, hh]
