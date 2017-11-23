import numpy as np
import time
import itertools
import multiprocessing as mp


def kernel(world, precisions, block, NUM_H_BINS):
    print("kernel started")
    dist_lookup = np.zeros((block[0][1] - block[0][0], block[1][1] - block[1][0], NUM_H_BINS))

    start_time = time.time()
    for x_bin in range(block[0][0], block[0][1]):
        print("{:.2f}% Complete - {:.2f}s ellapsed".format((x_bin - block[0][0]) / (block[0][1] - block[0][0]) * 100, time.time() - start_time))
        for y_bin in range(block[1][0], block[1][1]):
            for h_bin in range(NUM_H_BINS):
                x = x_bin * precisions[0]
                y = y_bin * precisions[1]
                h = h_bin * precisions[2]
                dist_lookup[x_bin - block[0][0], y_bin - block[1][0], h_bin] = world.get_rangefinder_distance(x,y,h)
    
    return dist_lookup

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

        # Do the computation in 4 blocks
        blocks = [((0, NUM_X_BINS // 2), (0, NUM_Y_BINS // 2)), ((0, NUM_X_BINS // 2), (NUM_Y_BINS // 2, NUM_Y_BINS)),
            ((NUM_X_BINS // 2, NUM_X_BINS), (0, NUM_Y_BINS // 2)), ((NUM_X_BINS // 2, NUM_X_BINS), (NUM_Y_BINS // 2, NUM_Y_BINS))]

        pool_it = tuple(zip(itertools.repeat(world, 4), itertools.repeat(precisions, 4), blocks, itertools.repeat(NUM_H_BINS, 4)))

        print("Starting kernels")
        with mp.Pool(4) as p:
            results = p.starmap(kernel, pool_it)
        
        for i,b in enumerate(blocks):
            dist_lookup[b[0][0]:b[0][1], b[1][0]:b[1][1], :] = results[i]

        return RangefinderCache(dist_lookup, precisions, bounds)

    def get(self, x, y, h):
        xx = int(round(x / self.precisions[0]))
        yy = int(round(y / self.precisions[1]))
        hh = int(round((float(h) % (np.pi * 2)) / self.precisions[2]))

        return self.data_cache[xx, yy, hh]
