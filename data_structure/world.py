import math

import numpy as np
import pygame

from constants import Colours, Units
from data_structure import get_distance
from display import Drawable


class World(Drawable):
    def __init__(self):
        # Minimal version of the maze.
        # self.maze = (
        #     (0, 0, 0, 0, 1, 0, 1, 0),
        #     (0, 0, 1, 0, 0, 0, 0, 0),
        #     (0, 1, 0, 1, 1, 0, 1, 0),
        #     (0, 0, 0, 0, 0, 0, 1, 0),
        # )
        self.maze = (
            (1, 1, 1, 1, 1, 1, 1, 1, 1, 1),
            (1, 0, 0, 0, 0, 1, 0, 1, 0, 1),
            (1, 0, 0, 1, 0, 0, 0, 0, 0, 1),
            (1, 0, 1, 0, 1, 1, 0, 1, 0, 1),
            (1, 0, 0, 0, 0, 0, 0, 1, 0, 1),
            (1, 1, 1, 1, 1, 1, 1, 1, 1, 1)
        )
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
        for block_num in range(31):
            if self.localization_squares[block_num] is None:
                continue
            for block_row_num, block_row in enumerate(self.localization_squares[block_num]):
                for block_col_num, block_value in enumerate(block_row):
                    x = (block_num % 8) * 4 + block_col_num
                    y = (block_num // 8) * 4 + block_row_num
                    self.maze_localization_grid[x, y] = block_value

        self.columns = len(self.maze[0])
        self.rows = len(self.maze)

    def draw(self, window):
        box_size_px = window.m_to_px(Units.METERS_IN_A_FOOT)
        for y, row in enumerate(self.maze):
            for x, cell in enumerate(row):
                if cell:
                    pygame.draw.rect(window.screen, Colours.BLACK,
                                     [x * box_size_px, y * box_size_px, box_size_px, box_size_px])

        code_size_px = window.m_to_px(Units.METERS_IN_A_FOOT) // 4
        ox, oy = box_size_px, box_size_px
        for x in range(4 * 8):
            for y in range(4 * 4):
                if self.maze_localization_grid[x, y]:
                    pygame.draw.rect(window.screen, Colours.GREY,
                                     [ox + x * code_size_px, oy + y * code_size_px, code_size_px, code_size_px])

    def get_rangefinder_distance(self, x, y, h):
        """
        :param x: Given in real-world units (meters)
        :param y: Given in real-world units (meters)
        :param h: Heading in degrees CW from North
        :return: distance that a perfect rangefinder would see
        """
        # We are inside a wall.
        if not self.is_free(x, y):
            return 0

        # TODO: Optimize this!
        # Loop through all obstacles and assume four edges for each.
        min_distance = math.inf
        for o_y, row in enumerate(self.maze):
            for o_x, cell in enumerate(row):
                if cell:
                    # For each obstacle:
                    # Top
                    o_x_1 = o_x * Units.METERS_IN_A_FOOT
                    o_y_1 = o_y * Units.METERS_IN_A_FOOT
                    o_x_2 = (o_x + 1) * Units.METERS_IN_A_FOOT
                    o_y_2 = o_y * Units.METERS_IN_A_FOOT
                    min_distance = min(min_distance,
                                       get_distance(p=(x, y), h=h,
                                                    l2=((o_x_1, o_y_1), (o_x_2, o_y_2))))
                    # Bottom
                    o_x_1 = o_x * Units.METERS_IN_A_FOOT
                    o_y_1 = (o_y + 1) * Units.METERS_IN_A_FOOT
                    o_x_2 = (o_x + 1) * Units.METERS_IN_A_FOOT
                    o_y_2 = (o_y + 1) * Units.METERS_IN_A_FOOT
                    min_distance = min(min_distance,
                                       get_distance(p=(x, y), h=h,
                                                    l2=((o_x_1, o_y_1), (o_x_2, o_y_2))))
                    # Left
                    o_x_1 = o_x * Units.METERS_IN_A_FOOT
                    o_y_1 = o_y * Units.METERS_IN_A_FOOT
                    o_x_2 = o_x * Units.METERS_IN_A_FOOT
                    o_y_2 = (o_y + 1) * Units.METERS_IN_A_FOOT
                    min_distance = min(min_distance,
                                       get_distance(p=(x, y), h=h,
                                                    l2=((o_x_1, o_y_1), (o_x_2, o_y_2))))
                    # Right
                    o_x_1 = (o_x + 1) * Units.METERS_IN_A_FOOT
                    o_y_1 = o_y * Units.METERS_IN_A_FOOT
                    o_x_2 = (o_x + 1) * Units.METERS_IN_A_FOOT
                    o_y_2 = (o_y + 1) * Units.METERS_IN_A_FOOT
                    min_distance = min(min_distance,
                                       get_distance(p=(x, y), h=h,
                                                    l2=((o_x_1, o_y_1), (o_x_2, o_y_2))))
        return min_distance

    def get_line_reading(self, x, y):
        """
        :param x: Given in real-world units (meters)
        :param y: Given in real-world units (meters)
        :return: True if the space has a dark-coloured floor.
        """
        if not self.is_free(x, y):
            return None
        return self.maze_localization_grid[
            math.floor((x - Units.METERS_IN_A_FOOT) / (Units.METERS_IN_A_FOOT / 4)), math.floor(
                (y - Units.METERS_IN_A_FOOT) / (Units.METERS_IN_A_FOOT / 4))]

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
