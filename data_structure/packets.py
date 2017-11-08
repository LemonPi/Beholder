class SensorReadingPacket(object):
    def __init__(self, robot_spec):
        self.NUM_BYTES = 4*(len(robot_spec.sensors)  + 1)
    
    def to_packet(self, senor)
