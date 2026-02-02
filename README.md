# Head Pitch & Yaw Control

The head is driven by **two Dynamixel motors** to control **pitch (nodding)** and **yaw (spinning)**.

### Motion intuition
If you look from the top:
- If the motors spin **away/towards each other**, the head **nods (pitch)**.
- If the motors spin **in the same direction**, the head **spins (yaw)**.

This script sets up both motors (operating mode, accel/velocity), captures the current pose as the "center", then asks the user for yaw/pitch angles and sync-writes both motors.

---

## Steps to move the head
1. Ensure both Dynamixels are mounted to the head mechanism
2. Daisy-chain the motors together (Motor 1 ↔ Motor 2)
3. Connect Motor 1 to the U2D2
4. Connect the U2D2 to computer/Jetson
5. Run the code

---

## Code explanation and what not

## Setup
1. Ensure python3 is installed  
   https://www.python.org/downloads/

2. Download Dynamixel SDK (source code from github)
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```
3. Build and install the SDK

- Open the `{Dynamixel_SDK}/python` folder  
- Open a terminal in that folder  
- Run:
```
pip install .
```
- Verify installation 
```
pip show dynamixel_sdk .
``` 
## head_pitch_yaw_control.py
This script controls two Dynamixels that together drive the head pitch and yaw.
When you run 
``` 
python head_pitch_yaw_control.py
```
- Opens the port and sets baudrate
- Reads operating modes of both motors
- Disables torque to allow configuration
- Sets operating mode (typically Extended Position Control)
- Sets acceleration and velocity profiles
- Enables torque
- Reads the current motor positions and treats them as the center pose
- Asks for yaw and pitch angles in degrees
- Converts angles into motor ticks
- Uses SyncWrite to move both motors simultaneously
- Disables torque and closes the port on exit

## Explanation
At the top we imported 
- dynamixel.sdk
- initialize.py (helper functions used throughout the script)

Communication is done by sending instruction packets (Tx) and receiving status packets (Rx) via the U2D2

## Constants
### Control table addresses
Used to read/write data to the motors
Addresses can be found in the Dynamixel control table or via Dynamixel Wizard 2.0

### Dynamixel IDs
- DXL_ID_1 = 1
- DXL_ID_2 = 2

Both motors: 
- Use protocol 2.0
- Are on the same bus

### Communication settings 
- Baudrate = 57600
- DEVICENAME = "COM4"

### Motion Parameters
- PROFILE_ACCELERATION
- PROFILE_VELOCITY
Higher values increase responsiveness but may stress the mechanism

### Gear constants
- TEETH_SMALL
- TEETH_LARGE
- GEAR_RATIO = TEETH_LARGE / TEETH_SMALL
Used to convert head angles into motor angles correctly

### Angle mapping constants
- COUNTS_PER_TURN
- COUNTS_PER_DEGREE
Used to convert degrees into raw Dynamixel ticks

### Control logic (high level)
1. Open port and set baudrate
2. Create port and packet handlers
3. Create SyncWrite group
4. Read operating modes
5. Disable torque
6. Set operating mode
7. Set acceleration and velocity
8. Enable torque
9. Capture current positions as center pose
10. Ask user for yaw and pitch
11. Convert angles → motor ticks
12. Sync-write goal positions
