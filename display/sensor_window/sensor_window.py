from collections import OrderedDict
from enum import Enum

import pygame

from constants import Colours
from display.sensor_window.line_data import LineData
from display.sensor_window.multiline_data import MultiLineData
from display.sensor_window.text_data import TextData

pygame.init()
pygame.font.init()


class DataType(Enum):
    LINE = 'line'
    MULTILINE = 'multiline'
    TEXT = 'text'


class SensorWindow(object):
    def __init__(self, text='', size=(400, 500)):
        self.screen = pygame.display.set_mode(size, pygame.SRCALPHA)
        pygame.display.set_caption(text)

        self._data = OrderedDict()

    def add_data_category(self, type, name, *args, **kwargs):
        # Ensure data category names are unique.
        if name in self._data.keys():
            raise KeyError('Names must be unique. {} was previously inserted as {}.'.format(name, self._data[name]))

        # Instantiate the data container for this data type.
        if type == DataType.LINE:
            self._data[name] = LineData(name, *args, **kwargs)
        elif type == DataType.MULTILINE:
            self._data[name] = MultiLineData(name, *args, **kwargs)
        elif type == DataType.TEXT:
            self._data[name] = TextData(name, *args, **kwargs)
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
        y = 0
        for k, v in self._data.items():
            surf = v.render()
            self.screen.blit(surf, (0, y))
            y += surf.get_size()[1] + 5


if __name__ == '__main__':
    update_clock = pygame.time.Clock()
    window = SensorWindow(text='Data streams')
    window.add_data_category(DataType.LINE, 'sawtooth', maxlen=100)
    window.add_data_category(DataType.MULTILINE, 'sawtooths', num_lines=2, maxlen=100)
    window.add_data_category(DataType.TEXT, 'state', max_chars=7)
    i = 0
    while True:
        i = i + 1
        if i == 100:
            i = 0
        window.update({'sawtooth': i, 'sawtooths': [100 - i, 100 + i]})
        window.update({'state': 'UNKNOWN'})
        window.draw()
        pygame.display.flip()
        update_clock.tick(30)
        print('FPS: {}'.format(update_clock.get_fps()))
