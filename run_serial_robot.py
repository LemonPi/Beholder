import random
import time
import sys

import bluetooth as bl
import multiprocessing as mpc
import numpy as np
import pygame
import serial
import pickle

from constants import Units
from data_structure import World, ParticleFilter, Particle, RobotSpec, FloorSensor, DistanceSensor, Packetizer, SensorRobot
from display import SimulatorWindow

# ---- PARTICLE FILTER PARAMS
N_PARTICLES = 250
POS_SIGMA = 0.02
H_SIGMA = 0.02
UPDATE_EVERY = 30

# ---- ROBOT SPEC
# ------------------------------------------------------------
# Define the robot we will be working with
sensors = [
    ('range', DistanceSensor(np.array([[0], [0.005]]), 0, sigma=0.1)),
    ('right', DistanceSensor(np.array([[0.005], [0]]), np.pi / 2, sigma=0.1)),
    ('left', DistanceSensor(np.array([[-0.005], [0]]), -np.pi / 2, sigma=0.1)),
    #('floor', FloorSensor(np.array([[0], [0]])))
    ]
robot_spec = RobotSpec(sensors)

# ------------------------------------------------------------

# ---- SERIAL CODE
# ------------------------------------------------------------
def serial_reader(spec, positions, headings, readings):
    sensor_packetizer = Packetizer(("I","f","f","I","I","I","B"))
    pose_packetizer = Packetizer(("f","f","f","I","B"))

    # SERIAL PARAMS
    port = '/dev/ttyUSB0'
    baud = 9600

    # Set up world and particle filter
    # Load the rangefinder cache
    with open('range_finder_cache.pkl', 'rb') as f:
        rangefinder_cache = pickle.load(f)
    print("Loaded rangefinder cache")

    world = World(rangefinder_cache=rangefinder_cache)

    # Create the particle filter
    pf = ParticleFilter(N_PARTICLES, POS_SIGMA, H_SIGMA)

    ser = serial.Serial(port, baud, timeout=None)
    ser.reset_input_buffer()

    update_num = 1

    last_packet = time.time()
    
    while(True):
        while(True):
            char_in = ser.read(size=1)
            # Wait for start character
            if (char_in == b'\xa1'):
                break
        
        # Read packet
        packet_time = time.time()
        buffer = ser.read(size=packet_parser.NUM_BYTES)
        identifier, d, dh, readings = packet_parser.from_packet(buffer)
        print(identifier, d, dh, readings, packet_time - last_packet)
        pf.update_particle_weights(robot_spec, [x/1000.0 for x in readings], world)
        pf.move_particles(d, dh)      # Move the robot according to the encoder readings
        
        if(update_num % UPDATE_EVERY == 0):
            pf.resample()

       
        # Shared memory
        positions[:] = list(pf.particle_pos.reshape(2*N_PARTICLES,))
        headings[:] = list(pf.particle_h)

        update_num += 1
        last_packet = packet_time

ROBOT_PACKET_START_BYTE = b'\xa1'
PC_PACKET_START_BYTE = b'\xf5'

# ---- BLUETOOTH CODE
# ------------------------------------------------------------
def bluetooth_reader(spec, shm_positions, shm_headings, shm_readings, flag):
    sensor_packetizer = Packetizer("IffIIIB")
    pose_packetizer = Packetizer("fffIB")

    # ---- BLUETOOTH SETUP
    SPP_UID = "00001101-0000-1000-8000-00805f9b34fb"
    # for this module only
    MAC_ADDR = "00:14:03:05:D1:AE"

    service_matches = bl.find_service(uuid=SPP_UID, address=MAC_ADDR)

    if len(service_matches) != 1:
        print(service_matches)
        raise RuntimeError("Cannot find a bluetooth service; is it paired?")

    bl_service = service_matches[0]

    port = bl_service["port"]
    name = bl_service["name"]
    host = bl_service["host"]

    print("connecting to \"%s\" on %s" % (name, host))

    # Create the client socket
    sock = bl.BluetoothSocket(bl.RFCOMM)
    sock.connect((host, port))
    sock.settimeout(0.5)
    try:
        while(True):
            _ = sock.recv(10000)     # Get rid of all trash
    except bl.btcommon.BluetoothError as e:
        pass

    sock.setblocking(True)

    # ---- PARTICLE FILTER SETUP
    with open('range_finder_cache.pkl', 'rb') as f:
        rangefinder_cache = pickle.load(f)
    print("Loaded rangefinder cache")

    world = World(rangefinder_cache=rangefinder_cache)

    # Create the particle filter
    pf = ParticleFilter(N_PARTICLES, POS_SIGMA, H_SIGMA)


    # ---- START COMMUNICATIONS

    # TODO: Init robot state
    # Create a packet to send back to the robot
    data = (0.0, 0.0, 0.0, 0, 250)      # Start packet
    packet = pose_packetizer.to_packet(data)

    # Send data to robot
    # for i in range(10):
    #     print("Sending start signal")
    #     sock.send(PC_PACKET_START_BYTE)
    #     sock.send(packet)
    #     time.sleep(1)

    print("Sending start signal")
    sock.send(PC_PACKET_START_BYTE)
    sock.send(packet)

    # TODO: Bidirectional handshake

    # Initialize everything
    update_num = 1
    last_packet = time.time()

    # ------------------------------------------------------------
    # ---- READ-UPDATE LOOP
    # ------------------------------------------------------------
    while(True):
        # Handle Keyboard Input
        if (flag.value == 1):
            start_packet = pose_packetizer.to_packet((0.0, 0.0, 0.0, 0, 250))
            sock.send(PC_PACKET_START_BYTE)
            sock.send(start_packet)
            flag.value = 0
        if (flag.value == 2):
            stop_packet = pose_packetizer.to_packet((0.0, 0.0, 0.0, 0, 251))
            sock.send(PC_PACKET_START_BYTE)
            sock.send(stop_packet)
            flag.value = 0

        # ---- READ PACKETS
        # ------------------------------------------------------------
        # Listen for packets from the robot        
        while(True):
            char_in = sock.recv(1)
            # print(char_in, ord(char_in), char_in[0])
            # Check the reset character
            # if (char_in == b'\xa2'):
            #     print('Resetting...')
            #     # Reset the particle filter
            #     update_num = 1
            #     pf = ParticleFilter(N_PARTICLES, POS_SIGMA, H_SIGMA)
            #     break
            # Wait for start character
            if (char_in == ROBOT_PACKET_START_BYTE):
                break
        
        # Read packet
        packet_time = time.time()
        buffer = b''
        for i in range(sensor_packetizer.NUM_BYTES):
            b = sock.recv(1)
            buffer += b

        # Parse Packet
        crc_correct, packet_contents = sensor_packetizer.from_packet(buffer) 
        (seq_num, d, dh, sl, sf, sr, behaviour) = packet_contents
        readings = (sf, sr, sl)
        print(crc_correct, seq_num, d, dh, readings, packet_time - last_packet)

        pf.update_particle_weights(robot_spec, [x/1000.0 for x in readings], world)
        pf.move_particles(d  / 1000.0, dh)        # Move the robot according to the encoder readings
        
        # Resample particles
        if (update_num % UPDATE_EVERY == 0):
            pf.resample()

        update_time = time.time()
        print(update_time - packet_time)

        # ---- UPDATE ROBOT
        # ------------------------------------------------------------
        # Get an estimate of the robot's position
        com_pos, com_h, com_uncertainty, is_converged = pf.get_pose_estimate(convergence_radius=0.4)

        # Create a packet to send back to the robot
        # intent = 252 if is_converged else 253
        intent = 252
        data = (float(com_pos[0]), float(com_pos[1]), float(com_h), seq_num, intent)
        packet = pose_packetizer.to_packet(data)
        
        # Send data to robot
        sock.send(PC_PACKET_START_BYTE)
        sock.send(packet)

        # Update shared memory
        shm_positions[:] = list(pf.particle_pos.reshape(2*N_PARTICLES,))
        shm_headings[:] = list(pf.particle_h)
        shm_readings[:] = [x/1000.0 for x in readings]

        update_num += 1
        last_packet = packet_time

# ---- SHARED MEMORY

# Share the robot position and heading
positions = mpc.Array('f', 2*N_PARTICLES)
headings = mpc.Array('f', N_PARTICLES)
readings = mpc.Array('f', len(robot_spec.sensors))
flag = mpc.Value('I')

#p = mpc.Process(target=serial_reader, args=(robot_spec, positions, headings))
p = mpc.Process(target=bluetooth_reader, args=(robot_spec, positions, headings, readings, flag))
p.start()
# ------------------------------------------------------------

# ---- VISUALIZATION
# ------------------------------------------------------------
window = SimulatorWindow(text='Robot simulation')

world = World()
world.add_to_window(window)

# Shell of the robot class for localized robots
robot = SensorRobot(robot_spec)

# Draw to screen.
should_quit = False
t = 0
update_clock = pygame.time.Clock()
com_pos, com_h, com_uncertainty, is_converged = np.array([0, 0]), 0, np.array([1, 1]), False

while not should_quit:
    t += 1
    # Limit fps.
    update_clock.tick(40)

    # ---- KEYBOARD INPUT
    keys = pygame.key.get_pressed()
    if keys[pygame.K_o]:
        print("Turning on")
        flag.value = 1
    if keys[pygame.K_p]:
        print("Turning off")
        flag.value = 2

    # Extract positions and headings
    particle_pos = np.array(positions[:]).reshape((2, N_PARTICLES))
    particle_h = np.array(headings[:])

    # FIND COM
    com_pos = np.mean(particle_pos, axis=1)
    com_h = np.mean(particle_h)
    com_uncertainty = np.std(particle_pos, axis=1)
    is_converged = np.all(2 * com_uncertainty < 0.4)    

    # ---- DRAW OBJECTS
    # ------------------------------------------------------------
    window.draw()    

    for i in range(N_PARTICLES):
        Particle.draw(window, particle_pos[:, i], particle_h[i])
    
    # Draw the center of mass position and heading
    Particle.draw_com(window, com_pos, com_h, com_uncertainty)
    
    # Set the variables on the robot
    robot.localized = is_converged
    robot.pos = np.reshape(com_pos, (2,1))
    robot.h = com_h
    robot.sensor_readings = readings[:]

    robot.draw(window)

    # Go ahead and update the screen with what we've drawn.
    # This MUST happen after all the other drawing commands.
    pygame.display.flip()
    # ------------------------------------------------------------

    # Process events.
    mouse_clicked = False
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            should_quit = True
        if event.type == pygame.MOUSEBUTTONUP:
            mouse_clicked = True
