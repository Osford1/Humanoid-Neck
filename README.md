# Humanoid neck — two-motor differential pitch and yaw

**Two Dynamixel servos drive one 2-DOF head: they spin the same way and the head yaws, they spin
opposite ways and it nods.** This is the bench code I wrote to drive that mechanism over a U2D2 from a
laptop, one command at a time from the terminal.

It is a lab script, not a library. There is no trajectory planner, no ROS node and no test suite; it is
the thing I used to prove the mechanism and the differential mapping worked before the neck went on the
robot.

## How the differential works

Hold the head in your hands and look down the axis:

| Motor 1 | Motor 2 | Head does |
| --- | --- | --- |
| turns +θ | turns +θ | **yaws** by θ (spins left/right) |
| turns +θ | turns −θ | **pitch** by θ (nods up/down) |

So every command is decomposed into two motor goals:

```python
goal1 = center1 + yaw_ticks + pitch_ticks
goal2 = center2 + yaw_ticks - pitch_ticks
```

That is `yaw_pitch_control_and_send()` in `initialize.py`. The two motors are written in one
`GroupSyncWrite` packet so they start on the same bus frame — writing them one after the other makes the
head twitch between the two commands.

**Draw the sign convention on the bench before you command anything.** The mapping above is what the
mechanism does with the current gear train; if you flip a motor's direction, yaw and pitch swap.

## Hardware

| Item | What I used |
| --- | --- |
| Servos | 2 × Dynamixel, Protocol 2.0, 4-byte goal position (XM/XH series control table) |
| IDs | `DXL_ID_1 = 1`, `DXL_ID_2 = 2` |
| Interface | U2D2, daisy-chained: motor 1 ↔ motor 2, motor 1 → U2D2 |
| Gears | 23-tooth pinion on each motor into a 40-tooth gear (ratio 1.739) |
| Baud | 57600 |
| Port | `COM4` on my machine — change it |

## Wiring

1. Mount both servos into the head mechanism.
2. Daisy-chain the two servos.
3. Connect servo 1 to the U2D2, U2D2 to the laptop.
4. Power the servos from their own supply, not from the U2D2's USB rail.

## Run it

```bash
pip install dynamixel-sdk          # or: git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
                                   #     cd DynamixelSDK/python && pip install .
python basicNeck.py
```

The script then:

1. opens the port at 57600 and reports success or exits,
2. reads the operating mode of both servos,
3. **asks you for the operating mode** — enter `4` (Extended Position Control). This is interactive
   because I wanted the other modes reachable without editing the file; answer `4` unless you have a
   reason not to,
4. turns torque off, sets mode, profile acceleration and profile velocity, turns torque on,
5. reads both servos' present position and calls that the **centre** — so `0 0` means "stay where you
   are", not "go to a fixed home",
6. prompts `yaw pitch >` in degrees, and keeps prompting until you type `q`,
7. on exit, disables torque on both servos and closes the port.

```text
yaw pitch > 30 0      # yaw 30° left/right
yaw pitch > 0 -15     # nod 15° down
yaw pitch > 20 10     # both at once
yaw pitch > q
```

Torque is left **off** while the prompt is open between commands (the goal registers hold, the servos
hold position only while energised), so support the head if it is not counterbalanced.

## Knobs

Everything worth changing is a module constant at the top of `basicNeck.py`:

| Constant | File | Default | What it does |
| --- | --- | --- | --- |
| `DXL_ID_1`, `DXL_ID_2` | `basicNeck.py` | 1, 2 | servo IDs |
| `BAUDRATE` | `basicNeck.py` | 57600 | bus speed; must match both servos |
| `DEVICENAME` | `basicNeck.py` | `"COM4"` | serial port (`/dev/ttyUSB0` on Linux) |
| `TEETH_SMALL`, `TEETH_LARGE` | `basicNeck.py` | 23, 40 | gear teeth; sets `GEAR_RATIO` |
| `PROFILE_ACCELERATION` | `basicNeck.py` | 20 | ramp-up; higher accelerates harder |
| `PROFILE_VELOCITY` | `basicNeck.py` | 200 | cruise speed; higher is faster |
| `COUNTS_PER_TURN` | `basicNeck.py` | 4096.0 | encoder counts per motor revolution |

Degrees → motor counts is `angle × GEAR_RATIO × COUNTS_PER_DEGREE`; at the defaults one head degree is
about 19.8 counts, so the smallest command the mechanism can resolve is well under a tenth of a degree.

## Troubleshooting (errors I actually hit)

| Symptom | Cause | Fix |
| --- | --- | --- |
| `failed to open the port` | wrong `DEVICENAME`, or the U2D2 is not enumerated | check Device Manager for the COM number; on Linux `ls /dev/ttyUSB*` |
| `Failed to change the baudrate` | servo baud ≠ `BAUDRATE` | set both servos to 57600 in Dynamixel Wizard |
| `[TxRxResult] There is no status packet!` | daisy chain reversed, or only one servo powered | servo 1 → U2D2, servo 2 chained off servo 1 |
| head snaps to a random pose on start | you answered `0 0` expecting a home pose | `0 0` holds the pose at launch; a centre is captured, not commanded |
| head twitches between two poses | goals written separately | use the sync write; do not add a second `write4ByteTxRx` in the loop |
| servos go slack | torque is off on exit | expected: the script disables torque before closing the port |

## Status

Worked on the bench with both servos and the head mechanism. Not tested on the assembled robot, no
motion smoothing, no limits check — commanding a large yaw from an extreme pose will drive a servo into
its mechanical stop. Add soft limits before this runs unattended.

<!-- TODO(owner): the README has no photo of the mechanism. Add one photo of the assembled head (servos,
gears and U2D2 visible) to docs/screenshots/, then reference it here. -->

## Files

| File | Purpose |
| --- | --- |
| `basicNeck.py` | main script: setup, centre capture, yaw/pitch prompt loop |
| `initialize.py` | shared helpers: torque, operating mode, profiles, gains, position read, sync write |

> The previous README referred to this script as `head_pitch_yaw_control.py`. The file in the repo is
> `basicNeck.py`; the old name was wrong.

## Credits

Built for the University of Melbourne MSP humanoid's neck mechanism — the mechanism and the gear train
are the team's, the control code here is mine. Servo API from
[ROBOTIS DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (Apache-2.0). The operating-mode
name table in `initialize.py` follows the Robotis control-table documentation.

MIT — see `LICENSE`.
