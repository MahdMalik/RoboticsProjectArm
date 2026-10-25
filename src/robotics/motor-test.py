import json
import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.motors_bus import MotorCalibration

# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------
PORT = "/dev/ttyACM0"
CALIBRATION_FILE = "include/robot-config.json"

MODE = "timed"          # "timed" = change position every INTERVAL s, "keys" = press a key
INTERVAL = 3.0          # seconds between commands in timed mode
CYCLES = 1              # how many times to run through Position 0 -> 1 -> 2
MOVE_DURATION = 1.5     # seconds the smooth motion takes
MOVE_STEPS = 50         # interpolation steps per move
TOLERANCE = 2.0         # how close (deg / gripper units) counts as "arrived"
TIMEOUT = 3.0           # extra seconds to wait for arrival after the motion ends
POLL_DELAY = 0.01       # seconds between position reads

MOTORS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]

# Required joint configurations (degrees, gripper 0-100)
POSITIONS = [
    {   # Position 0
        "shoulder_pan": 0.0,
        "shoulder_lift": -30.0,
        "elbow_flex": 30.0,
        "wrist_flex": 70.0,
        "wrist_roll": 0.0,
        "gripper": 0.0,
    },
    {   # Position 1
        "shoulder_pan": 45.0,
        "shoulder_lift": -20.0,
        "elbow_flex": 30.0,
        "wrist_flex": 60.0,
        "wrist_roll": -30.0,
        "gripper": 50.0,
    },
    {   # Position 2
        "shoulder_pan": -45.0,
        "shoulder_lift": -10.0,
        "elbow_flex": 0.0,
        "wrist_flex": -30.0,
        "wrist_roll": 30.0,
        "gripper": 30.0,
    },
]

# ------------------------------------------------------------
# Setup
# ------------------------------------------------------------
bus = FeetechMotorsBus(
    port=PORT,
    motors={
        "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
    },
)


def load_calibration(path):
    with open(path, "r") as f:
        data = json.load(f)
    return {
        name: MotorCalibration(
            id=v["id"],
            drive_mode=v["drive_mode"],
            homing_offset=v["homing_offset"],
            range_min=v["range_min"],
            range_max=v["range_max"],
        )
        for name, v in data.items()
    }


def read_positions():
    return {m: bus.read("Present_Position", m) for m in MOTORS}


def at_target(current, target):
    return all(abs(current[m] - target[m]) <= TOLERANCE for m in MOTORS)


def move_smooth_and_time(target, duration=MOVE_DURATION, steps=MOVE_STEPS):
    """
    Smoothly interpolate (smoothstep: slow -> fast -> slow) from the
    current position to the target, then wait until every joint is
    within TOLERANCE. Returns (elapsed_seconds, final_positions, reached).
    The timer starts when the command begins and stops on arrival.
    """
    starts = read_positions()
    t0 = time.perf_counter()

    for i in range(steps + 1):
        t = i / steps
        eased = t * t * (3 - 2 * t)

        for m in MOTORS:
            position = starts[m] + (target[m] - starts[m]) * eased
            bus.write("Goal_Position", m, position)

        time.sleep(duration / steps)

    # Motion finished sending; wait for the arm to settle at the target
    while True:
        current = read_positions()
        elapsed = time.perf_counter() - t0

        if at_target(current, target):
            return elapsed, current, True

        if elapsed > duration + TIMEOUT:
            return elapsed, current, False

        time.sleep(POLL_DELAY)


def print_result(index, elapsed, final, target, reached):
    print(f"\n=== Position {index} ===")
    status = "reached" if reached else "NOT reached (timed out)"
    print(f"Target {status}")
    print(f"Time to reach target: {elapsed:.3f} s")
    print(f"{'Joint':<15}{'Target':>10}{'Final':>10}")
    for m in MOTORS:
        print(f"{m:<15}{target[m]:>10.1f}{final[m]:>10.2f}")


def wait_for_next(last_command_time):
    """Returns False if the user asked to quit."""
    if MODE == "timed":
        remaining = INTERVAL - (time.perf_counter() - last_command_time)
        if remaining > 0:
            time.sleep(remaining)
        return True

    # keyboard mode (Windows): SPACE/ENTER = next, Q = quit
    import msvcrt
    print("\nPress SPACE for the next position, Q to quit...")
    while True:
        key = msvcrt.getwch().lower()
        if key in (" ", "\r"):
            return True
        if key == "q":
            return False


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------
if len(sys.argv) > 1 and sys.argv[1] in ("timed", "keys"):
    MODE = sys.argv[1]

bus.connect()
bus.calibration = load_calibration(CALIBRATION_FILE)

try:
    for m in MOTORS:
        bus.write("Operating_Mode", m, OperatingMode.POSITION.value)
        bus.write("Torque_Enable", m, 1)

    print(f"Mode: {MODE} | Cycles: {CYCLES} (Ctrl+C to stop)")

    sequence = [i for _ in range(CYCLES) for i in range(len(POSITIONS))]
    quit_early = False

    for n, index in enumerate(sequence):
        target = POSITIONS[index]
        print(f"\nCommanding Position {index}...")

        command_time = time.perf_counter()
        elapsed, final, reached = move_smooth_and_time(target)
        print_result(index, elapsed, final, target, reached)

        is_last = n == len(sequence) - 1
        if not is_last and not wait_for_next(command_time):
            quit_early = True
            break

    print("\nStopped early." if quit_early else "\nSequence complete!")

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    for m in MOTORS:
        bus.write("Torque_Enable", m, 0)
    bus.disconnect()
    print("Disconnected.")
