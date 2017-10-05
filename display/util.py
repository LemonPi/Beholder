import math

import pygame


def draw_triangle(screen, x, y, h, r, c, aspect_ratio=3, com_triangle=2 / 3, fill=False):
    h_rad = math.radians(h)
    dx = math.sin(h_rad)
    dy = -math.cos(h_rad)
    px = -dy
    py = dx

    centering_x = -dx * r * aspect_ratio / 2 * com_triangle
    centering_y = -dy * r * aspect_ratio / 2 * com_triangle

    pygame.draw.polygon(screen, c,
                        [[round(x + r * px + centering_x),
                          round(y + r * py + centering_y)],
                         [round(x - r * px + centering_x),
                          round(y - r * py + centering_y)],
                         [round(x + aspect_ratio * r * dx + centering_x),
                          round(y + aspect_ratio * r * dy + centering_y)]], 0 if fill else 1)


def true_until_window_closed():
    for event in pygame.event.get():  # User did something
        if event.type == pygame.QUIT:  # If user clicked close
            return False
    return True


def yield_until_window_closed():
    while True:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                return False
        yield False


def block_until_window_closed():
    while True:
        for event in pygame.event.get():  # User did something
            if event.type == pygame.QUIT:  # If user clicked close
                return
