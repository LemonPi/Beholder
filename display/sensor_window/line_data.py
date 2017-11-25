from collections import deque

import matplotlib
import pygame

from .data_category import DataCategory

matplotlib.use("Agg")

import matplotlib.backends.backend_agg as agg
import pylab


class LineData(DataCategory):
    def __init__(self, name, maxlen=None):
        assert maxlen is not None, 'maxlen is required for Line data.'
        super().__init__(name)
        self._data = deque([0] * maxlen, maxlen=maxlen)
        self.min = 0
        self.max = 1
        self.fig = pylab.figure(figsize=[4, 2],  # Inches
                                dpi=100,  # 100 dots per inch, so the resulting buffer is 400x200 pixels
                                )
        pylab.title(name)
        self.ax = self.fig.gca()
        self.line1, = self.ax.plot(self._data)

    def update(self, new_data):
        self._data.append(new_data)
        self.min = min(self.min, new_data)
        self.max = max(self.max, new_data)
        self.line1.set_ydata(self._data)

    def render(self):
        self.ax.set_ylim(bottom=self.min, top=self.max)
        canvas = agg.FigureCanvasAgg(self.fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        return surf
