import pygame
import numpy as np

from constants import Colours, Units

pygame.init()


class SimulatorWindow(object):
    def __init__(self, text='', size=800, aspect_ratio=8 / 4, real_world_size=(8, 4)):
        self.aspect_ratio = aspect_ratio
        self.real_world_size = real_world_size
        self.screen = pygame.display.set_mode([size, int(size / aspect_ratio)], pygame.SRCALPHA)
        assert self.screen.get_size()[0] == aspect_ratio * self.screen.get_size()[1]
        pygame.display.set_caption(text)

        self.cuid = 0
        self.drawables = {}

    def add_drawable(self, drawable):
        self.cuid += 1
        uid = self.cuid
        self.drawables[uid] = drawable
        return uid

    def remove_drawable(self, uid):
        return self.drawables.pop(uid)

    def m_to_px(self, m):
        width, height = self.screen.get_size()
        m_to_px_ratio = width / (Units.METERS_IN_A_FOOT * self.real_world_size[0])
        return np.rint(m * m_to_px_ratio).astype(int)

    def draw(self):
        # Clear the screen and set the screen background
        self.screen.fill(Colours.WHITE)

        for uid, d in self.drawables.items():
            d.draw(self)
