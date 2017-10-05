class Robot(object):
    def __init__(self, world, x, y, h):
        super().__init__(world, x, y, h)

    def read_sensor_outputs(self):
        """
        Returns a tuple of sensor readings from the robot. (x_1, ... x_n)
        """
        return (0,)

    def draw(self, window):
        pass
