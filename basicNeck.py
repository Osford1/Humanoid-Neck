''' Two motors to control the pitch and yaw of the motor
If you look from the top, if motors spin away or towards each other
the head nods.
If they spin in the same direction, the head spins
'''

from dynamixel_sdk import *
import initialize as init
# We are now using two dynamixels with the same protocol 2.0
# Control table addresses
TORQUE_ENABLE_ADDRESS               = 64
GOAL_POSITION_ADDRESS               = 116
PRESENT_POSITION_ADDRESS            = 132
OPERATING_MODE_ADDRESS              = 11
PROFILE_ACCELERATION_ADDRESS        = 108
PROFILE_VELOCITY_ADDRESS            = 112
POSITION_P_GAIN_ADDRESS             = 84
POSITION_I_GAIN_ADDRESS             = 82
POSITION_D_GAIN_ADDRESS             = 80

# Dynamixel IDs 
DXL_ID_1                      = 1
DXL_ID_2                      = 2

# Protocol version
PROTOCOL_VERSION              = 2.0

# General Settings 
BAUDRATE                    = 57600
DEVICENAME                  = "COM4"       # Check which port is being used on your controller

# Other Constants
data                        = 1            # Data you want to write to the specified address
TORQUE_ON                   = 1
TORQUE_OFF                  = 0

# acceleration / velocity constants
PROFILE_ACCELERATION = 20           # to be changed 
PROFILE_VELOCITY     = 200

# gear constants
TEETH_SMALL                 = 23
TEETH_LARGE                 = 40

GEAR_RATIO                  = TEETH_LARGE/TEETH_SMALL
CENTER_POS = 0
COUNTS_PER_TURN             = 4096.0
COUNTS_PER_DEGREE           = COUNTS_PER_TURN/360.0        # the resolution




# Initialize handler objects 
# Port handler object
portHandler = PortHandler(DEVICENAME)
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("failed to open the port")
    exit()
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    exit()

# Packet Handler object 
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Group object
# I want to write to register GOAL_POSITION_ADDRESS 
group = GroupSyncWrite(portHandler, packetHandler, GOAL_POSITION_ADDRESS, 4)        # data size is 4 bits in Protocol 2.0

# configure the motors


# Read both the operating modes
init.readOperatingMode(packetHandler,portHandler, DXL_ID_1, OPERATING_MODE_ADDRESS)       # dxl 1
init.readOperatingMode(packetHandler,portHandler, DXL_ID_2, OPERATING_MODE_ADDRESS)       # dxl 2

# Disable the torque to set up the operating modes and what not 
packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, TORQUE_ENABLE_ADDRESS, TORQUE_OFF)  # dxl 1
packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, TORQUE_ENABLE_ADDRESS, TORQUE_OFF)  # dxl 2

print("disabling torque to set operating mode on both dynamixels")

# set the operating mode
init.setOperatingMode(packetHandler,portHandler, DXL_ID_1, OPERATING_MODE_ADDRESS)        # dxl 1
init.setOperatingMode(packetHandler,portHandler, DXL_ID_2, OPERATING_MODE_ADDRESS)        # dxl 2

# set the acceleration profile
init.setAccelerationProfile(packetHandler,portHandler, DXL_ID_1, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION)  # dxl 1
init.setAccelerationProfile(packetHandler,portHandler, DXL_ID_2, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION)  # dxl 2

# set the velocity profile
init.setVelocityProfile(packetHandler, portHandler, DXL_ID_1, PROFILE_VELOCITY_ADDRESS,PROFILE_VELOCITY)        # dxl 1
init.setVelocityProfile(packetHandler, portHandler, DXL_ID_2, PROFILE_VELOCITY_ADDRESS,PROFILE_VELOCITY)        # dxl 2

print("setting velocity and acceleration profiles")

# set the PID gains later

# Enable torque
packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, TORQUE_ENABLE_ADDRESS, TORQUE_ON)       # dxl 1
packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, TORQUE_ENABLE_ADDRESS, TORQUE_ON)       # dxl 2


# Setup complete ^^^^

# establish "center from current positions"
# (This makes yaw = 0, pitch = 0 mean "hold current pose")
# note that yaw is spining, pitch is noding 

center1 = init.read_current_position(packetHandler, portHandler, DXL_ID_1, PRESENT_POSITION_ADDRESS)
center2 = init.read_current_position(packetHandler, portHandler, DXL_ID_2, PRESENT_POSITION_ADDRESS)

print(f"Center Positions captureed")
print(f"ID {DXL_ID_1} : {center1}")
print(f"ID {DXL_ID_2} : {center2}")

# control loop: ask for yaw and pitch angles in degrees 
# convert to motor ticks and sync-write both 
init.yaw_pitch_control_and_send(group, packetHandler, GEAR_RATIO, COUNTS_PER_DEGREE,
                               center1, center2, DXL_ID_1, DXL_ID_2)


# turn off the torque? 
packetHandler.write1ByteTxRx(portHandler, DXL_ID_1, TORQUE_ENABLE_ADDRESS, TORQUE_OFF)
packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, TORQUE_ENABLE_ADDRESS, TORQUE_OFF)

portHandler.closePort()
print("Done, Torque is turnt the fuck off, port is closed")
