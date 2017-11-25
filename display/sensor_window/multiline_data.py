from collections import deque

import matplotlib
import matplotlib.pyplot as plt
import pygame

from .data_category import DataCategory

matplotlib.use("Agg")

import matplotlib.backends.backend_agg as agg
import pylab
import numpy as np


class MultiLineData(DataCategory):
    def __init__(self, name, line_names=None, maxlen=None):
        assert line_names is not None, 'line_names is required for MultiLine data.'
        assert maxlen is not None, 'maxlen is required for MultiLine data.'
        super().__init__(name)
        self.num_lines = len(line_names)

        self._data = [deque([0] * maxlen, maxlen=maxlen) for _ in range(self.num_lines)]
        self.min = 0
        self.max = 1
        self.fig = pylab.figure(figsize=[4, 2],  # Inches
                                dpi=100,  # 100 dots per inch, so the resulting buffer is 400x200 pixels
                                )
        pylab.title(name)
        self.ax = self.fig.gca()
        self.lines = self.ax.plot(np.array(self._data).T)
        plt.legend(self.lines, line_names, loc='upper left')

    def update(self, new_data):
        assert len(new_data) == self.num_lines, 'Incorrect number of data points! {} required but {} passed.'.format(
            self.num_lines, len(new_data))
        for i in range(self.num_lines):
            self._data[i].append(new_data[i])
            self.min = min(self.min, new_data[i])
            self.max = max(self.max, new_data[i])
            self.lines[i].set_ydata(self._data[i])

    def render(self):
        self.ax.set_ylim(bottom=self.min, top=self.max)
        canvas = agg.FigureCanvasAgg(self.fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        return surf
