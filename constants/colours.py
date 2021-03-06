class Colours(object):
    # Define the colors we will use in RGB format
    BLACK = (0, 0, 0)
    GREY = (100, 100, 100)
    WHITE = (255, 255, 255)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)

    # Constants
    WALL_COLOUR = BLACK
    ROBOT_COLOUR = BLUE
    PARTICLE_COLOUR = RED
    COM_UNCERTAINTY_UNCERTAIN = (*RED, 128)
    COM_UNCERTAINTY_CONVERGED = (*GREEN, 128)
    COM_COLOUR = GREEN
