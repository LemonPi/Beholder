import pygame

from .data_category import DataCategory

pygame.font.init()
font = pygame.font.SysFont('Comic Sans MS', 30)


class TextData(DataCategory):
    def __init__(self, name, max_chars=None):
        assert max_chars is not None, 'max_chars is required for Text data.'
        self.max_chars = max_chars
        super().__init__(name)
        self._data = str(None)

    def update(self, new_data):
        self._data = new_data

    def render(self):
        text = '{}: {}'.format(self.name, self._data if len(self._data) <= self.max_chars else (
            self._data[:self.max_chars - 3] + '...'))
        surface = font.render(text, False, (0, 0, 0))
        return surface
