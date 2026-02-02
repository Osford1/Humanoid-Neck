# Function to set up motor before getting user input and moving 
# openning the port and setting baudrate not included here
from dynamixel_sdk import*

# function to enable torque with error detection 
def enable_Torque(packetHandler, portHandler, DXL_ID, TORQUE_ENABLE_ADDRESS, data):

    
    # write1ByteTxRx returns a communication result. We can check the communication result and error via :

    dxl_communication_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, TORQUE_ENABLE_ADDRESS, data)

    if dxl_communication_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

# Function to read operating mode 
def readOperatingMode(packetHandler, portHandler, DXL_ID, OPERATING_MODE_ADDRESS):
    # Code to read the operating mode: 
    operating_mode, dxl_communication_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, DXL_ID, OPERATING_MODE_ADDRESS)

    # error handling for reading operating mode
    if dxl_communication_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:        # that means theres an error
        print(packetHandler.getRxPacketError(dxl_error))
    else:
        print("Operating mode value:", operating_mode)      # prints the operating mode

        # Decode the operaeting mode (curtesy of ChatGpt)
        mode_names = {
            0: "Current Control Mode",
            1: "Velocity Control Mode",
            3: "Position Control Mode",
            4: "Extended Position Control Mode",
            5: "Current-based position control mode",
            16: "PWM Control mode"
        }

        print("Current Mode:", mode_names.get(operating_mode, "Unknown"))

# function to set operating mode
def setOperatingMode(packetHandler, portHandler, DXL_ID, OPERATING_MODE_ADDRESS):

    # Get the user input for operating mode
    while True:
        try:
            desired_operating_mode = int(input("Enter the operating mode desired:"))
            break                           # once we got a valid operating mode number we get tf outta the loop 
        # Not an integer
        except ValueError:
            print("Please enter an integer: 0, 1, 3, 4, 5, 16")
            continue

    # Setting the operating mode 
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, OPERATING_MODE_ADDRESS, desired_operating_mode)
    print("Setting the operating mode to: ", desired_operating_mode)

# function to set the motion profile
def setAccelerationProfile(packetHandler, portHandler, DXL_ID, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION):
    
    print("Setting the motion profile (accel/velocity)")
    # Acceleration
    dxl_communication_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, PROFILE_ACCELERATION_ADDRESS, PROFILE_ACCELERATION)
    if dxl_communication_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))

def setVelocityProfile(packetHandler, portHandler, DXL_ID, PROFILE_VELOCITY_ADDRESS, PROFILE_VELOCITY):
    # veloctiy 
    dxl_communication_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, PROFILE_VELOCITY_ADDRESS, PROFILE_VELOCITY)
    if dxl_communication_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))

# functions to set PID gains 
def setPosition_P_gain(packetHandler, portHandler, DXL_ID, POSITION_P_GAIN_ADDRESS, POSITION_P_GAIN):
    # P gain
    dxl_communication_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, POSITION_P_GAIN_ADDRESS, POSITION_P_GAIN)
    if dxl_communication_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))

def setPosition_I_gain(packetHandler, portHandler, DXL_ID, POSITION_I_GAIN_ADDRESS, POSITION_I_GAIN):
    # I gain 
    dxl_communication_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, POSITION_I_GAIN_ADDRESS, POSITION_I_GAIN)
    if dxl_communication_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))

def setPosition_D_gain(packetHandler, portHandler, DXL_ID, POSITION_D_GAIN_ADDRESS, POSITION_D_GAIN):
    # D gain
    dxl_communication_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, POSITION_D_GAIN_ADDRESS, POSITION_D_GAIN)
    if dxl_communication_result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(dxl_communication_result))
    elif dxl_error != 0:
        print(packetHandler.getRxPacketError(dxl_error))


# function to run motor in extended position control mode
def extended_position_control(packetHandler, portHandler, DXL_ID, PRESENT_POSITION_ADDRESS, COUNTS_PER_DEGREE, GEAR_RATIO, MOTOR_PERIOD_DEG, GOAL_POSITION_ADDRESS):
    '''POSITION CONTROL MODE '''
    # Get user input for the target position
    while True:
        try:
            # getting an input angle (plate) 
            plate_angle_deg = float(input("Enter angle (0 - 360, -9999999 to exit): "))
            if plate_angle_deg == -9999999:
                target_position = -9999999

            else:
                # Read the current position once to help with finding the shortest path 
                present_position, dxl_communication_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, PRESENT_POSITION_ADDRESS)
                if dxl_communication_result != COMM_SUCCESS:
                    print(packetHandler.getTxRxResult(dxl_communication_result))
                    continue
                elif dxl_error != 0:
                    print(packetHandler.getRxPacketError(dxl_error))
                    continue
                # current motor angle in degrees (multi-turn)
                motor_angle_current_deg = present_position / COUNTS_PER_DEGREE

                # desired motor angle for this plate angle 
                motor_angle_desired_deg = plate_angle_deg * GEAR_RATIO

                # find an integer k such that motor_angle_des + k * MOTOR_PERDIOD_DEG is closest to motor_angle_cur_deg
                k = round((motor_angle_current_deg - motor_angle_desired_deg)/ MOTOR_PERIOD_DEG)
                
                motor_angle_target = motor_angle_desired_deg + k * MOTOR_PERIOD_DEG

                # Convert target motor angle to counts
                raw_goal = int(motor_angle_target * COUNTS_PER_DEGREE)
                print("Current motor angle: ", motor_angle_current_deg)
                print("Target Motor angle: ", motor_angle_target)
                print("Target goal (counts): ", raw_goal)

                target_position = raw_goal
        # YOU DID NOT GIVE ME AN INTEGER
        except ValueError:
            print("Please enter an integer")
            continue                # repeat the loop 

        if target_position == -9999999:
            break
        
        # Write the target position to the Dynamixel    
        # Write4ByteTxRx returns a communication result. We can check the communication result and error via:
        dxl_communication_result , dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, GOAL_POSITION_ADDRESS, target_position)

        if dxl_communication_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_communication_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # raed the current position from the dynamixel 
        while True:
            present_position, dxl_communication_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, PRESENT_POSITION_ADDRESS)
        
            if dxl_communication_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_communication_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            print(f"Current Position: {present_position}")

            # threshold 
            if abs(target_position - present_position) <= 10:
                break


# Helpers for Sync write 
def _u32_to_le_bytes(val: int):
    return[
        DXL_LOBYTE(DXL_LOWORD(val)),
        DXL_HIBYTE(DXL_LOWORD(val)),
        DXL_LOBYTE(DXL_HIWORD(val)),
        DXL_HIBYTE(DXL_HIWORD(val))
    ]

# Function to safely read the current positon of the motor
def read_current_position(packetHandler, portHandler, dxl_id, PRESENT_POSITION_ADDRESS):
    pos, comm, err = packetHandler.read4ByteTxRx(portHandler, dxl_id, PRESENT_POSITION_ADDRESS)
    if comm != COMM_SUCCESS:
        raise RuntimeError(packetHandler.getTxRxResult(comm))
    if err != 0:
        raise RuntimeError(packetHandler.getPacketError(err))
    return pos

# function to write data into the group 
def sync_write_goal_position(group, packetHandler, goals_dict):
    """
    goals_dict : { DXL_ID_1: goal 1, DXL_ID_2 : goal 2}
    """

    group.clearParam()
    for dxl_id, goal in goals_dict.items():
        ok = group.addParam(dxl_id, _u32_to_le_bytes(int(goal)))
        if not ok:
            raise RuntimeError(f"Failed to addParam for ID {dxl_id}")
    comm = group.txPacket()
    if comm != COMM_SUCCESS:
        raise RuntimeError(packetHandler.getTxRxResult(comm))
    

# yaw and pitch control
def yaw_pitch_control_and_send(group, packetHandler, gear_ratio, counts_per_degree,
                               center1, center2, id1, id2):
    import time

    print("Enter: yaw_deg pitch_deg   (or q to quit)")
    while True:
        s = input("yaw pitch > ").strip()
        if s.lower() in ("q", "quit", "exit"):
            break

        parts = s.split()
        if len(parts) != 2:
            print("Please enter two numbers: yaw_deg pitch_deg")
            continue

        try:
            yaw_deg = float(parts[0])
            pitch_deg = float(parts[1])
        except ValueError:
            print("Please enter valid numbers.")
            continue

        yaw_ticks   = yaw_deg   * gear_ratio * counts_per_degree
        pitch_ticks = pitch_deg * gear_ratio * counts_per_degree

        goal1 = int(center1 + yaw_ticks + pitch_ticks)
        goal2 = int(center2 + yaw_ticks - pitch_ticks)

        print(f"Goals (ticks): ID{id1}={goal1}, ID{id2}={goal2}")

        # SEND immediately (sync write)
        goals = {id1: goal1, id2: goal2}
        sync_write_goal_position(group, packetHandler, goals)

        # give the motors time to start moving
        time.sleep(0.2)
