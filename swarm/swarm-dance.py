# ------------------------- INITIALIZATION -------------------------- #
# importing libraries
from djitellopy import TelloSwarm
import time

# --- this code assumes a specific diamond configuration which is presented in the attached pdf
d = 50  # distance increment
h = 80  # default height
v = 30  # default speed

# --- initialize tello modes
# the function below read the IPS of the tellos
# on the next line, input each IP of your Tellos as shown below in the form of strings under a list
swarm = TelloSwarm.fromIps(["192.168.0.73", "192.168.0.74", "192.168.0.75", "192.168.0.162"])
time.sleep(1)
swarm.connect()
time.sleep(1)
swarm.parallel(lambda i, tello: tello.enable_mission_pads())
swarm.parallel(lambda i, tello: tello.set_mission_pad_detection_direction(0))


# ------------------------- FUNCTIONS TO PERFORM -------------------------- #
# --- function to rotate clockwise in inner square
def rotate_inner_clock(i, tello):
    id = tello.get_mission_pad_id()
    if id == 1:
        tello.go_xyz_speed_yaw_mid(0, -2*d, h, v, 0, 1, 2)
    elif id == 2:
        tello.go_xyz_speed_yaw_mid(-2*d, 0, h, v, 0, 2, 3)
    elif id == 3:
        tello.go_xyz_speed_yaw_mid(0, 2*d, h, v, 0, 3, 4)
    elif id == 4:
        tello.go_xyz_speed_yaw_mid(2*d, 0, h, v, 0, 4, 1)
    swarm.sync()

# --- function to rotate anticlockwise in inner square
def rotate_inner_anti(i, tello):
    id = tello.get_mission_pad_id()
    if id == 1:
        tello.go_xyz_speed_yaw_mid(-2*d, 0, h, v, 0, 1, 4)
    elif id == 4:
        tello.go_xyz_speed_yaw_mid(0, -2*d, h, v, 0, 4, 3)
    elif id == 3:
        tello.go_xyz_speed_yaw_mid(2*d, 0, h, v, 0, 3, 2)
    elif id == 2:
        tello.go_xyz_speed_yaw_mid(0, 2*d, h, v, 0, 2, 1)
    swarm.sync()

# --- function to go from square to vertices
def go_inner_to_outer(i, tello):
    id = tello.get_mission_pad_id()
    if id == 1:
        tello.go_xyz_speed_yaw_mid(d, -d, h, v, 0, 1, 5)
    elif id == 2:
        tello.go_xyz_speed_yaw_mid(-d, -d, h, v, 0, 2, 6)
    elif id == 3:
        tello.go_xyz_speed_yaw_mid(-d, d, h, v, 0, 3, 7)
    elif id == 4:
        tello.go_xyz_speed_yaw_mid(d, d, h, v, 0, 4, 8)
    swarm.sync()

# --- function to go back to square from the vertices
def go_outer_to_inner(i, tello):
    id = tello.get_mission_pad_id()
    if id == 5:
        tello.go_xyz_speed_yaw_mid(-d, -d, h, v, 0, 5, 2)
    elif id == 6:
        tello.go_xyz_speed_yaw_mid(-d, d, h, v, 0, 6, 3)
    elif id == 7:
        tello.go_xyz_speed_yaw_mid(d, d, h, v, 0, 7, 4)
    elif id == 8:
        tello.go_xyz_speed_yaw_mid(d, -d, h, v, 0, 8, 1)
    swarm.sync()

# --- function to rotate clockwise in the vertices
def rotate_outer_clock(i, tello):
    id = tello.get_mission_pad_id()
    if id == 5:
        tello.go_xyz_speed_yaw_mid(-2*d, -2*d, h, v, 0, 5, 6)
    elif id == 6:
        tello.go_xyz_speed_yaw_mid(-2*d, 2*d, h, v, 0, 6, 7)
    elif id == 7:
        tello.go_xyz_speed_yaw_mid(2*d, 2*d, h, v, 0, 7, 8)
    elif id == 8:
        tello.go_xyz_speed_yaw_mid(2*d, -2*d, h, v, 0, 8, 5)
    swarm.sync()

# --- function to rotate anticlockwise in the vertices
def rotate_outer_anti(i, tello):
    id = tello.get_mission_pad_id()
    if id == 5:
        tello.go_xyz_speed_yaw_mid(-2*d, 2*d, h, v, 0, 5, 8)
    elif id == 8:
        tello.go_xyz_speed_yaw_mid(-2*d, -2*d, h, v, 0, 8, 7)
    elif id == 7:
        tello.go_xyz_speed_yaw_mid(2*d, -2*d, h, v, 0, 7, 6)
    elif id == 6:
        tello.go_xyz_speed_yaw_mid(2*d, 2*d, h, v, 0, 6, 5)
    swarm.sync()

# --- function to swap opposing vertices
def swap_opposing(i, tello):
    id = tello.get_mission_pad_id()
    if id == 5:
        tello.go_xyz_speed_yaw_mid(-4*d, 0, h+30, v, 0, 5, 7)
    elif id == 7:
        tello.go_xyz_speed_yaw_mid(4*d, 0, h, v, 0, 7, 5)
    swarm.sync()
    if id == 6:
        tello.go_xyz_speed_yaw_mid(0, 4*d, h, v, 0, 6, 8)
    elif id == 8:
        tello.go_xyz_speed_yaw_mid(0, -4*d, h+30, v, 0, 8, 6)

# --- function to swap drones diagonally
def swap_diagonally(i, tello):
    id = tello.get_mission_pad_id()
    if id == 1:
        tello.go_xyz_speed_yaw_mid(-2*d, -2*d, h+30, v, 0, 1, 3)
    elif id == 3:
        tello.go_xyz_speed_yaw_mid(2*d, 2*d, h, v, 0, 3, 1)
    swarm.sync()
    if id == 2:
        tello.go_xyz_speed_yaw_mid(-2*d, 2*d, h+30, v, 0, 2, 4)
    elif id == 4:
        tello.go_xyz_speed_yaw_mid(2*d, -2*d, h, v, 0, 4, 2)
    swarm.sync()

# --- function to make the drones bounce
def bounce(i, tello):
    id = tello.get_mission_pad_id()
    if id == 1 or id == 3 or id == 6 or id == 8:
        tello.go_xyz_speed_mid(0, 0, h+30, v, id)
        tello.go_xyz_speed_mid(0, 0, h-20, v, id)
        tello.go_xyz_speed_mid(0, 0, h+30, v, id)
        tello.go_xyz_speed_mid(0, 0, h-20, v, id)
        tello.go_xyz_speed_mid(0, 0, h, v, id)
    else:
        tello.go_xyz_speed_mid(0, 0, h -20, v, id)
        tello.go_xyz_speed_mid(0, 0, h +30, v, id)
        tello.go_xyz_speed_mid(0, 0, h -20, v, id)
        tello.go_xyz_speed_mid(0, 0, h +30, v, id)
        tello.go_xyz_speed_mid(0, 0, h, v, id)
    swarm.sync()

# ------------------------- CHOREOGRAPHY -------------------------- #
# --- all the possible functions that can be achieved are presented in the attached pdf
# --- the choreography below is just an example, feel free to add and remove steps to the choreography
# --- when adding additional sequences, ensure that the drones will start on the mission pads required for
# that specific sequence
# --- you can perform one of these functions by typing swarm.parallel(name-of-function), using the functions
# written above (the examples can be seen below)
# --- if there seems to be delay between the drones, might be okay to add delays in the form of time.sleep(delay-time)
swarm.parallel(lambda i, tello: tello.takeoff())
time.sleep(.5)
swarm.parallel(rotate_inner_anti)
swarm.parallel(go_inner_to_outer)
swarm.parallel(rotate_outer_clock)
swarm.parallel(bounce)
swarm.parallel(swap_opposing)
swarm.parallel(go_outer_to_inner)
swarm.parallel(swap_diagonally)
swarm.land()

