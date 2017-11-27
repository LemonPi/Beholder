import time
from typing import NamedTuple
import struct
import functools
import operator

from .sensors import Sensor

def compute_crc(buffer):
    return functools.reduce(operator.xor, buffer)

class RobotSpec(object):
    def __init__(self, sensors_names):
        self.SensorTuple = NamedTuple('SensorTuple', [(name, Sensor) for name, s in sensors_names])
        self.SensorReadingTuple = NamedTuple('SensorReadingTuple', [(name, float) for name, s in sensors_names])

        self.sensors = self.SensorTuple(*[s for n, s in sensors_names])

class Packetizer(object):
    def __init__(self, fmt_string):
        self._base_struct = struct.Struct(fmt_string)        
        self._crc_struct = struct.Struct(fmt_string + "B")
        self.NUM_BYTES = self._crc_struct.size
    
    def to_packet(self, values):
        base_buffer = self._base_struct.pack(*values)
        return self._crc_struct.pack(*(values + (compute_crc(base_buffer),)))        
    
    def from_packet(self, buffer):
        my_crc = compute_crc(buffer[:-1])
        results = self._crc_struct.unpack(buffer)
        crc = results[-1]
        return (crc == my_crc), results[:-1]

def main():
    # ---- ROBOT SPEC
    # ------------------------------------------------------------
    # Define the robot we will be working with
    sensors = [
        ('range', DistanceSensor([0, 0], 0, sigma=0.1)),
        ('right', DistanceSensor([0, 0], np.pi/2, sigma=0.1)),
        ('left', DistanceSensor([0, 0], -np.pi/2, sigma=0.1)),
        ('floor', FloorSensor([0, 0]))]
    robot_spec = RobotSpec(sensors)
    # ------------------------------------------------------------

    pacman = SensorReadingPacketizer(robot_spec)

    # Create a sample sensor reading
    readings = robot_spec.SensorReadingTuple(1.0, 2.0, 3.0, 4.0)
    packet = pacman.to_packet(readings)
    print(pacman.from_packet(packet))

if __name__ == '__main__':
    main()
