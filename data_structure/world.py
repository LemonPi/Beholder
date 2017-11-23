import math

import numpy as np
import pygame

from constants import Colours, Units
from data_structure import get_distance, get_distance_rectilinear
from display import Drawable

import time
import pickle


class World(Drawable):
    def __init__(self, rangefinder_cache=None):
        self.maze = (
            (0, 0, 0, 0, 1, 0, 1, 0),
            (0, 0, 1, 0, 0, 0, 0, 0),
            (0, 1, 0, 1, 1, 0, 1, 0),
            (0, 0, 0, 0, 0, 0, 1, 0),
        )
        self.walls = (
            ((1,2),(5,2)),     # CENTER OBSTACLE
            ((1,2),(1,3)), 
            ((1,3),(2,3)), 
            ((2,3),(2,1)), 
            ((2,1),(3,1)), 
            ((1,3),(2,3)),
            ((3,1),(3,3)),
            ((3,1),(3,3)),
            ((3,3),(5,3)),
            ((3,2),(5,2)),
            ((5,2),(5,3)),
            ((4,0),(4,1)),      # CENTER TOP OBSTACLE
            ((4,1),(5,1)),
            ((5,1),(5,0)),
            ((6,0),(6,1)),      # TOP RIGHT OBSTACLE
            ((6,1),(7,1)),
            ((7,1),(7,0)),
            ((6,2),(6,4)),      # BOTTOM RIGHT OBSTACLE
            ((6,2),(7,2)),
            ((7,2),(7,4)))
        self.localization_squares = {
            0: ((0, 1, 1, 0), (0, 0, 1, 0), (1, 0, 1, 1), (0, 0, 0, 0)),
            1: ((0, 1, 0, 1), (0, 0, 1, 0), (0, 0, 1, 1), (1, 0, 1, 0)),
            2: ((1, 1, 0, 0), (0, 1, 1, 0), (1, 1, 1, 0), (0, 1, 1, 1)),
            3: ((1, 0, 0, 1), (1, 0, 0, 1), (1, 1, 1, 0), (1, 1, 1, 1)),
            4: None,
            5: ((1, 1, 0, 1), (1, 1, 0, 0), (0, 1, 0, 0), (0, 1, 0, 1)),
            6: None,
            7: ((0, 1, 0, 0), (1, 0, 0, 0), (0, 1, 0, 1), (1, 0, 0, 0)),
            8: ((0, 0, 0, 0), (1, 1, 1, 1), (1, 0, 0, 1), (0, 0, 1, 1)),
            9: ((1, 0, 1, 0), (1, 1, 0, 0), (0, 0, 1, 1), (1, 0, 0, 1)),
            10: None,
            11: ((0, 1, 1, 0), (0, 1, 1, 0), (0, 0, 0, 1), (0, 1, 1, 0)),
            12: ((1, 0, 0, 0), (0, 1, 1, 1), (0, 0, 1, 0), (0, 1, 1, 0)),
            13: ((1, 0, 1, 1), (1, 0, 1, 1), (0, 1, 0, 1), (1, 0, 1, 1)),
            14: ((0, 0, 0, 1), (1, 0, 1, 0), (0, 0, 1, 0), (0, 1, 1, 1)),
            15: ((1, 1, 1, 0), (1, 0, 0, 0), (1, 1, 1, 0), (0, 0, 1, 1)),
            16: ((0, 0, 1, 0), (0, 0, 1, 0), (0, 0, 1, 1), (1, 0, 0, 0)),
            17: None,
            18: ((0, 0, 0, 0), (0, 0, 1, 1), (0, 1, 1, 1), (1, 1, 1, 0)),
            19: None,
            20: None,
            21: ((0, 1, 1, 0), (0, 1, 0, 1), (1, 0, 0, 1), (1, 0, 0, 0)),
            22: None,
            23: ((1, 1, 0, 1), (0, 1, 1, 1), (1, 1, 1, 0), (1, 0, 0, 0)),
            24: ((0, 0, 1, 0), (1, 1, 1, 1), (0, 0, 1, 0), (1, 1, 0, 0)),
            25: ((1, 0, 0, 0), (1, 0, 0, 0), (0, 0, 1, 1), (1, 1, 1, 1)),
            26: ((0, 1, 0, 0), (0, 0, 1, 1), (1, 1, 1, 0), (0, 1, 1, 0)),
            27: ((1, 1, 1, 0), (0, 0, 1, 1), (0, 1, 1, 1), (0, 1, 0, 0)),
            28: ((1, 0, 1, 1), (0, 0, 0, 0), (1, 0, 1, 1), (0, 0, 1, 1)),
            29: ((1, 1, 1, 1), (0, 0, 1, 1), (1, 1, 1, 0), (1, 0, 0, 1)),
            30: None,
            31: ((1, 0, 0, 1), (0, 0, 1, 0), (1, 1, 0, 0), (1, 0, 1, 0))
        }
        
        # (x, y) maps to colour (0=white, 1=black)
        self.maze_localization_grid = np.zeros([4 * 8, 4 * 4])
        for block_num in range(max(self.localization_squares.keys()) + 1):
            if self.localization_squares[block_num] is None:
                continue
            for block_row_num, block_row in enumerate(self.localization_squares[block_num]):
                for block_col_num, block_value in enumerate(block_row):
                    x = (block_num % 8) * 4 + block_col_num
                    y = (block_num // 8) * 4 + block_row_num
                    self.maze_localization_grid[x, y] = block_value

        self.columns = len(self.maze[0])
        self.rows = len(self.maze)

        self.rangefinder_cache = rangefinder_cache

    def draw(self, window):
        box_size_px = window.m_to_px(Units.METERS_IN_A_FOOT)
        for y, row in enumerate(self.maze):
            for x, cell in enumerate(row):
                if cell:
                    pygame.draw.rect(window.screen, Colours.BLACK,
                                     [x * box_size_px, y * box_size_px, box_size_px, box_size_px])

        code_size_px = window.m_to_px(Units.METERS_IN_A_FOOT) // 4
        for x in range(4 * 8):
            for y in range(4 * 4):
                if self.maze_localization_grid[x, y]:
                    pygame.draw.rect(window.screen, Colours.GREY,
                                     [x * code_size_px, y * code_size_px, code_size_px, code_size_px])

    def get_rangefinder_distance(self, x, y, h):
        """
        :param x: Given in real-world units (meters)
        :param y: Given in real-world units (meters)
        :param h: Heading in degrees CW from North
        :return: distance that a perfect rangefinder would see
        """

        if not self.is_free(x, y):
            return 0
        
        if (self.rangefinder_cache):
            return self.rangefinder_cache.get(x,y,h)

        min_distance = math.inf
        # For map edge:
        # Top
        min_distance = min(min_distance,
                           get_distance(p=(x, y), h=h,
                                        l2=((0, 0), (8 * Units.METERS_IN_A_FOOT, 0))))
        # Bottom
        min_distance = min(min_distance,
                           get_distance(p=(x, y), h=h,
                                        l2=((0, 4 * Units.METERS_IN_A_FOOT),
                                            (8 * Units.METERS_IN_A_FOOT, 4 * Units.METERS_IN_A_FOOT))))
        # Left
        min_distance = min(min_distance,
                           get_distance(p=(x, y), h=h,
                                        l2=((0, 0), (0, 4 * Units.METERS_IN_A_FOOT))))
        # Right
        min_distance = min(min_distance,
                           get_distance(p=(x, y), h=h,
                                        l2=((8 * Units.METERS_IN_A_FOOT, 0),
                                            (8 * Units.METERS_IN_A_FOOT, 4 * Units.METERS_IN_A_FOOT))))

        for p1,p2 in self.walls:
            p1s = tuple((s * Units.METERS_IN_A_FOOT) for s in p1)
            p2s = tuple((s * Units.METERS_IN_A_FOOT) for s in p2)
            min_distance = min(min_distance,
                                get_distance(p=(x, y), h=h,
                                            l2=(p1s, p2s)))
        
        # TODO: Make the rectilinear thing work.
        # pos = np.array([[x], [y]])
        # headings = np.array([h])
        # for p1, p2 in self.walls:
        #     p1s = tuple((s * Units.METERS_IN_A_FOOT) for s in p1)
        #     p2s = tuple((s * Units.METERS_IN_A_FOOT) for s in p2)
        #     walls = np.expand_dims(np.transpose(np.array([p1s, p2s])), axis=2)
            
        #     min_distance = min(min_distance,
        #                         float(get_distance_rectilinear(p=pos, h=h,
        #                                     l2=walls)))

        return min_distance

    def get_line_reading(self, pos):
        """
        :param x: Given in real-world units (meters)
        :param y: Given in real-world units (meters)
        :return: True if the space has a dark-coloured floor.
        """
        if not self.is_free(pos[0], pos[1]):
            return None
        return self.maze_localization_grid[
            math.floor(pos[0] / (Units.METERS_IN_A_FOOT / 4)),
            math.floor(pos[1] / (Units.METERS_IN_A_FOOT / 4))]

    def get_width_m(self):
        return len(self.maze[0]) * Units.METERS_IN_A_FOOT

    def get_height_m(self):
        return len(self.maze) * Units.METERS_IN_A_FOOT

    def is_in(self, x, y):
        """
        :param x: Given in units of the maze grid (feet)
        :param y: Given in units of the maze grid (feet)
        :return: True if the position is within the map.
        """
        if x < 0 or y < 0 or x >= self.columns or y >= self.rows:
            return False
        return True

    def is_free(self, x, y):
        """
        :param x: Given in real-world units (meters)
        :param y: Given in real-world units (meters)
        :return: True if the space is both inside the map and not occupied by an obstacle.
        """
        r_x, r_y = math.floor(x / Units.METERS_IN_A_FOOT), math.floor(y / Units.METERS_IN_A_FOOT)
        if not self.is_in(r_x, r_y):
            return False
        return self.maze[r_y][r_x] == 0
