import random
import time
import sys

import bluetooth as bl
import multiprocessing as mpc
import numpy as np
import pygame
import serial
import pickle

from constants import Units, Intents, PacketCodes
from data_structure import World, ParticleFilter, Particle, RobotSpec, FloorSensor, DistanceSensor, Packetizer, SensorRobot, Navigator, world_to_block_tuple
from display import SimulatorWindow

# ---- PARTICLE FILTER PARAMS
N_PARTICLES = 250
POS_SIGMA = 0.035
H_SIGMA = 0.03
UPDATE_EVERY = 25

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
            # otherwise not in the middle of the packet and we can assume it's debugging
            else :
                print(chr(char_in[0]), end='')
        
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
ANY_HEADING = 1000.0

# ---- BLUETOOTH CODE
# ------------------------------------------------------------
def build_command_packet(pacman, intent):
    return pacman.to_packet((0.0, 0.0, 0.0, 0, intent))

def send_packet(sock, packet):
    sock.send(PC_PACKET_START_BYTE)
    sock.send(packet)

def enable_navigation_mode(sock, pose_packetizer):
    for i in range(5):
        packet = build_command_packet(pose_packetizer, Intents.DISABLE_WALL_FOLLOW)
        send_packet(sock, packet)
        time.sleep(0.1)

    for i in range(5):
        packet = build_command_packet(pose_packetizer, Intents.DISABLE_WALL_TURN)
        send_packet(sock, packet)
        time.sleep(0.1)

def bluetooth_reader(spec, shm_positions, shm_headings, shm_readings, flag, shm_state):
    sensor_packetizer = Packetizer("IffIIIB")
    pose_packetizer = Packetizer("fffIB")
    
    START_PACKET = build_command_packet(pose_packetizer, Intents.START)
    STOP_PACKET = build_command_packet(pose_packetizer, Intents.STOP)
    CLEAR_PACKET = build_command_packet(pose_packetizer, Intents.CLEAR)

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

    # 1 - Turn the robot off
    send_packet(sock, STOP_PACKET)

    sock.settimeout(0.5)
    try:
        while(True):
            _ = sock.recv(1024)     # Get rid of all trash            
    except bl.btcommon.BluetoothError as e:
        pass

    # Partially Non-blocking solution
    sock.settimeout(0.1)

    # ---- PARTICLE FILTER SETUP
    with open('range_finder_cache.pkl', 'rb') as f:
        rangefinder_cache = pickle.load(f)
    print("Loaded rangefinder cache")

    world = World(rangefinder_cache=rangefinder_cache)

    # Create the particle filter and navigator
    pf = ParticleFilter(N_PARTICLES, POS_SIGMA, H_SIGMA)
    nav = Navigator(world.maze)

    # ---- START COMMUNICATIONS

    # time.sleep(10)

    # Create a packet to send back to the robot
    send_packet(sock, START_PACKET)

    # TODO: Bidirectional handshake

    # Initialize everything
    update_num = 1
    last_packet = time.time()

    # ------------------------------------------------------------
    # ---- READ-UPDATE LOOP
    # ------------------------------------------------------------
    while (True):
        # Handle Keyboard Input
        if (flag.value == 1):
            send_packet(sock, START_PACKET)
            flag.value = 0
        if (flag.value == 2):
            send_packet(sock, STOP_PACKET)
            flag.value = 0
        if (flag.value == 3):
            update_num = 1
            pf = ParticleFilter(N_PARTICLES, POS_SIGMA, H_SIGMA)
            flag.value = 0
        if (flag.value == 4):
            send_packet(sock, build_command_packet(pose_packetizer, Intents.CLOSE_CLAW))
            flag.value = 0
        if (flag.value == 5):
            send_packet(sock, build_command_packet(pose_packetizer, Intents.OPEN_CLAW))            
            flag.value = 0

        # ---- READ PACKETS
        # ------------------------------------------------------------
        # Listen for packets from the robot
        char_in = b''
        try:
            char_in = sock.recv(1)
        except bl.btcommon.BluetoothError as e:
            pass
        
        # Wait for start character
        if (char_in != PacketCodes.ROBOT_PACKET_START):
            continue

        # Read packet
        packet_time = time.time()
        buffer = b''
        for i in range(sensor_packetizer.NUM_BYTES):
            b = ''
            while b == '':
                # Wait for something to come through Bluetooth.
                b = sock.recv(1)
            buffer += b

        # Parse Packet
        crc_correct, packet_contents = sensor_packetizer.from_packet(buffer)
        (seq_num, d, dh, sl, sf, sr, behaviour) = packet_contents
        readings = (sf, sr, sl)
        print(crc_correct, seq_num, behaviour, d, dh, readings, packet_time - last_packet)
        shm_state.value = behaviour

        pf.update_particle_weights(robot_spec, [x/1000.0 for x in readings], world)
        pf.move_particles(d  / 1000.0, dh)        # Move the robot according to the encoder readings
        
        # Resample particles
        if (update_num % UPDATE_EVERY == 0):
            pf.resample()

        update_time = time.time()

        # ---- UPDATE ROBOT
        # ------------------------------------------------------------
        # Get an estimate of the robot's position
        com_pos, com_h, com_uncertainty, is_converged = pf.get_pose_estimate(convergence_radius=0.4)

        # TODO: Make robot go get block
        if (nav.target is None and is_converged):
            print("Sending nav data")
            # packet = pose_packetizer.to_packet((0.5 * 1000.0 * Units.METERS_IN_A_FOOT, 0.5 * 1000.0 * Units.METERS_IN_A_FOOT, 0, 0, Intents.POSE_UPDATE))
            # send_packet(sock, packet)
            send_packet(sock, STOP_PACKET)

            nav.update(com_pos, np.array([0.5, 2.5]) * Units.METERS_IN_A_FOOT)
            
            pickup_loc = np.array([0.5, 1.7]) * Units.METERS_IN_A_FOOT
            
            end_loc = np.array([2.5, 2.5]) * Units.METERS_IN_A_FOOT

            # Send waypoints
            send_packet(sock, CLEAR_PACKET)
            enable_navigation_mode(sock, pose_packetizer)
            for i,p in enumerate(nav.get_waypoints()):
                # TODO: Specify headings for the get and put targets.                
                data = (p[0,0] * 1000.0, p[1,0] * 1000.0, ANY_HEADING, 0, Intents.ADD_WAYPOINT)
                packet = pose_packetizer.to_packet(data)
                send_packet(sock, packet)
                time.sleep(0.1)
            
            data = (pickup_loc[0] * 1000.0, pickup_loc[1] * 1000.0, 0.0, 0, Intents.GET_BLOCK)
            packet = pose_packetizer.to_packet(data)
            send_packet(sock, packet)
            time.sleep(0.1)

            data = (end_loc[0] * 1000.0, end_loc[1] * 1000.0, 0.0, 0, Intents.PUT_BLOCK)
            packet = pose_packetizer.to_packet(data)
            send_packet(sock, packet)
            time.sleep(0.1)

            send_packet(sock, START_PACKET)

        # Create a packet to send back to the robot
        intent = Intents.POSE_UPDATE #if is_converged else Intents.POSE_PING
        data = (float(com_pos[0]) * 1000, float(com_pos[1])  * 1000, float(com_h), seq_num, intent)        
        packet = pose_packetizer.to_packet(data)
        # send_packet(sock, packet)

        # Update shared memory
        shm_positions[:] = list(pf.particle_pos.reshape(2*N_PARTICLES,))
        shm_headings[:] = list(pf.particle_h)
        shm_readings[:] = [x / 1000.0 for x in readings]

        update_num += 1
        last_packet = packet_time

# ---- SHARED MEMORY

# Share the robot position and heading
positions = mpc.Array('f', 2*N_PARTICLES)
headings = mpc.Array('f', N_PARTICLES)
readings = mpc.Array('f', len(robot_spec.sensors))
flag = mpc.Value('I')
state = mpc.Value('I')

#p = mpc.Process(target=serial_reader, args=(robot_spec, positions, headings))
p = mpc.Process(target=bluetooth_reader, args=(robot_spec, positions, headings, readings, flag, state))
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
    if keys[pygame.K_r]:
        print("Resetting particle filter")
        flag.value = 3
    if keys[pygame.K_q]:
        flag.value = 4
    if keys[pygame.K_w]:
        flag.value = 5

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
    robot.state = state.value
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
