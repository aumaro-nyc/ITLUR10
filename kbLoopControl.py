# This macro allows moving a robot using the keyboard
# Note: This works on console mode only, you must run the PY file separately
#
# More information about the RoboDK API here:
# https://robodk.com/doc/en/RoboDK-API.html
# Type help("robolink") or help("robodk") for more information

from robolink import *    # API to communicate with RoboDK
from robodk import *      # basic matrix operations
import threading
RDK = Robolink()

# Arrow keys program example

# get a robot
robot = RDK.Item('', ITEM_TYPE_ROBOT)
if not robot.Valid():
    print("No robot in the station. Load a robot first, then run this program.")
    pause(5)
    raise Exception("No robot in the station!")

print('Using robot: %s' % robot.Name())
print('Use the arrows (left, right, up, down), Q and A keys to move the robot')
print('Note: This works on console mode only, you must run the PY file separately')

RUN_ON_ROBOT = True

# Important: by default, the run mode is RUNMODE_SIMULATE
# If the program is generated offline manually the runmode will be RUNMODE_MAKE_ROBOTPROG,
# Therefore, we should not run the program on the robot
if RDK.RunMode() != RUNMODE_SIMULATE:
    RUN_ON_ROBOT = False

if RUN_ON_ROBOT:
    # Update connection parameters if required:
    # robot.setConnectionParams('192.168.2.35',30000,'/', 'anonymous','')
    
    # Connect to the robot using default IP
    success = robot.Connect() # Try to connect once
    #success robot.ConnectSafe() # Try to connect multiple times
    status, status_msg = robot.ConnectedState()
    if status != ROBOTCOM_READY:
        # Stop if the connection did not succeed
        print(status_msg)
        raise Exception("Failed to connect: " + status_msg)
    
    # This will set to run the API programs on the robot and the simulator (online programming)
    RDK.setRunMode(RUNMODE_RUN_ROBOT)
    # Note: This is set automatically when we Connect() to the robot through the API

#else:
    # This will run the API program on the simulator (offline programming)
    # RDK.setRunMode(RUNMODE_SIMULATE)
    # Note: This is the default setting if we do not execute robot.Connect()
    # We should not set the RUNMODE_SIMULATE if we want to be able to generate the robot programm offline

# define the move increment
move_speed = 10
new_robot_joints = robot.Joints()

def update_robot():
    while True:
        robot.MoveJ(new_robot_joints)

def getInput():
    from msvcrt import getch
    while True:
        key = (ord(getch()))
        move_direction = [0,0,0]
        # print(key)
        if key == 75:
            print('arrow left (Y-)')
            move_direction = [0,-1,0]
        elif key == 77:
            print('arrow right (Y+)')
            move_direction = [0,1,0]
        elif key == 72:
            print('arrow up (X-)')
            move_direction = [-1,0,0]
        elif key == 80:
            print('arrow down (X+)')
            move_direction = [1,0,0]
        elif key == 113:
            print('Q (Z+)')
            move_direction = [0,0,1]
        elif key == 97:
            print('A (Z-)')
            move_direction = [0,0,-1]

        # make sure that a movement direction is specified
        if norm(move_direction) <= 0:
            continue

        # calculate the movement in mm according to the movement speed
        xyz_move = mult3(move_direction, move_speed)

        # get the robot joints
        robot_joints = robot.Joints()

        # get the robot position from the joints (calculate forward kinematics)
        robot_position = robot.SolveFK(robot_joints)

        # get the robot configuration (robot joint state)
        robot_config = robot.JointsConfig(robot_joints)

        # calculate the new robot position
        new_robot_position = transl(xyz_move)*robot_position

        # calculate the new robot joints
        new_robot_joints = robot.SolveIK(new_robot_position)
        if len(new_robot_joints.tolist()) < 6:
            print("No robot solution!! The new position is too far, out of reach or close to a singularity")
            continue

        # calculate the robot configuration for the new joints
        new_robot_config = robot.JointsConfig(new_robot_joints)

        if robot_config[0] != new_robot_config[0] or robot_config[1] != new_robot_config[1] or robot_config[2] != new_robot_config[2]:
            print("Warning!! Robot configuration changed!! This will lead to unextected movements!")
            print(robot_config)
            print(new_robot_config)

        # move the robot joints to the new position
       # robot.MoveJ(new_robot_joints)
        #robot.MoveL(new_robot_joints)

try:
    kbThread = threading.Thread(target=getInput)
    updateThread = threading.Thread(target=update_robot)

    kbThread.start()
    updateThread.start()
except Exception as e:
    print(e)

#try:
#    thread.start_new_thread(getInput)
#    thread.start_new_thread(update_robot)
#except:
#    print("Error! Unable to start threads")