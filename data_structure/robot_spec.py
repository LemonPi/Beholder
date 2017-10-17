from typing import NamedTuple

from .sensors import Sensor


class RobotSpec(object):
    def __init__(self, sensors_names):
        self.SensorTuple = NamedTuple('SensorTuple', [(name, Sensor) for name, s in sensors_names])
        self.SensorReadingTuple = NamedTuple('SensorReadingTuple', [(name, float) for name, s in sensors_names])

        self.sensors = self.SensorTuple(*[s for n, s in sensors_names])
