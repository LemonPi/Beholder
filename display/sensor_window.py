from collections import deque
from enum import Enum

import pygame

from constants import Colours

pygame.init()
pygame.font.init()

import matplotlib

matplotlib.use("Agg")

import matplotlib.backends.backend_agg as agg
import pylab


# TODO(wheung): Move this to a constants enum.
class DataType(Enum):
    LINE = 'line'


class DataCategory(object):
    def __init__(self, name):
        self.name = name

    def update(self, new_data):
        """Accepts new data for the data category."""
        raise NotImplementedError('Data category did not implement update!')

    def render(self):
        """Returns a PyGame Surface that can be blitted onto the window."""
        raise NotImplementedError('Data category did not implement draw!')


class LineData(DataCategory):
    def __init__(self, name, maxlen):
        assert maxlen is not None, 'maxlen is required for Line data.'
        super().__init__(name)
        self._data = deque([0] * maxlen, maxlen=maxlen)
        self.fig = pylab.figure(figsize=[4, 4],  # Inches
                                dpi=100,  # 100 dots per inch, so the resulting buffer is 400x400 pixels
                                )
        pylab.title(name)
        self.ax = self.fig.gca()
        self.line1, = self.ax.plot(self._data)

    def update(self, new_data):
        self._data.append(new_data)

    def render(self):
        self.line1.set_ydata(self._data)
        # TODO(wheung): Can optimize this by keeping min and max of circular buffer.
        self.ax.set_ylim(bottom=min(self._data), top=max(self._data))
        canvas = agg.FigureCanvasAgg(self.fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        return surf


# TODO(wheung): Implement MultiLine DataCategory

class SensorWindow(object):
    def __init__(self, text='', size=(400, 500)):
        self.screen = pygame.display.set_mode(size, pygame.SRCALPHA)
        pygame.display.set_caption(text)

        self._data = {}

    def add_data_category(self, type, name, *args, **kwargs):
        if type == DataType.LINE:
            self._data[name] = LineData(type, name, *args, **kwargs)
        else:
            raise NotImplementedError('Invalid data category type.')

    def delete_data_category(self, name):
        self._data.pop(name)

    def update(self, new_values):
        """
        Update the data. You must call draw afterwards to render the graphics.
        :param kwargs: (key->value), where key is the name of any existing data category.
        :return:
        """
        for k, v in new_values.items():
            self._data[k].update(v)

    def draw(self):
        # Clear the screen and set the screen background
        self.screen.fill(Colours.WHITE)
        for k, v in self._data.items():
            self.screen.blit(v.render(), (0, 0))


if __name__ == '__main__':
    update_clock = pygame.time.Clock()
    window = SensorWindow(text='Data streams')
    window.add_data_category(DataType.LINE, 'sawtooth', maxlen=100)
    i = 0
    while True:
        i = i + 1
        if i == 100:
            i = 0
        window.update({'sawtooth': i})
        window.draw()
        pygame.display.flip()
        update_clock.tick(30)
