import time
from typing import NamedTuple
import struct

from .sensors import Sensor


class RobotSpec(object):
    def __init__(self, sensors_names):
        self.SensorTuple = NamedTuple('SensorTuple', [(name, Sensor) for name, s in sensors_names])
        self.SensorReadingTuple = NamedTuple('SensorReadingTuple', [(name, float) for name, s in sensors_names])

        self.sensors = self.SensorTuple(*[s for n, s in sensors_names])

class SensorReadingPacketizer(object):
    def __init__(self, robot_spec):
        format_string = "IIff" + "I"*len(robot_spec.sensors)
        self.struct = struct.Struct(format_string)
        self.NUM_BYTES = self.struct.size
    
    def to_packet(self, t, d, dh, sensor_readings):
        return self.struct.pack(t, d, dh, *sensor_readings)
    
    def from_packet(self, buffer):
        results = self.struct.unpack(buffer)
        return results[0], results [1], results[2], results[3], results[4:]

class PoseUpdatePacketizer(object):
    def __init__(self):
        format_string = "II3f"
        self.struct = struct.Struct(format_string)
        self.NUM_BYTES = self.struct.size
    
    def to_packet(self, seq_num, packet_t, x, y, h):
        return self.struct.pack(seq_num, packet_t, x, y, h)
    
    def from_packet(self, buffer):
        results = self.struct.unpack(buffer)
        return tuple(results)

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
