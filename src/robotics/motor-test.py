import json
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode
from lerobot.motors.motors_bus import MotorCalibration


PORT = "COM46"
CALIBRATION_FILE = "include/robot-config.json"


# ------------------------------------------------------------
# Motor setup
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


# ------------------------------------------------------------
# Load calibration
# ------------------------------------------------------------

with open(CALIBRATION_FILE, "r") as f:
    data = json.load(f)

calibration = {
    name: MotorCalibration(
        id=values["id"],
        drive_mode=values["drive_mode"],
        homing_offset=values["homing_offset"],
        range_min=values["range_min"],
        range_max=values["range_max"],
    )
    for name, values in data.items()
}


# ------------------------------------------------------------
# Connect
# ------------------------------------------------------------

bus.connect()
bus.calibration = calibration


motors = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


# Position mode + torque
for motor in motors:
    bus.write(
        "Operating_Mode",
        motor,
        OperatingMode.POSITION.value,
    )

    bus.write(
        "Torque_Enable",
        motor,
        1,
    )


# ------------------------------------------------------------
# Motion helpers
# ------------------------------------------------------------

def get_positions():
    return {
        motor: bus.read("Present_Position", motor)
        for motor in motors
    }


def move_smooth(targets, duration=1.0, steps=40):
    """
    Smoothly move from the current position to the target.

    Smoothstep:
        slow -> fast -> slow
    """

    starts = get_positions()

    for i in range(steps + 1):

        t = i / steps

        # Smoothstep easing
        eased = t * t * (3 - 2 * t)

        for motor, target in targets.items():

            start = starts[motor]

            position = start + (target - start) * eased

            bus.write(
                "Goal_Position",
                motor,
                position,
            )

        time.sleep(duration / steps)


def hold(seconds):
    time.sleep(seconds)


# ------------------------------------------------------------
# Poses
# ------------------------------------------------------------

HOME = {
    "shoulder_pan": 0,
    "shoulder_lift": 0,
    "elbow_flex": 0,
    "wrist_flex": 0,
    "wrist_roll": 0,
    "gripper": 20,
}


# Negative shoulder lift raises the arm on your setup.
RAISED = {
    "shoulder_lift": -25,
    "elbow_flex": -20,
    "wrist_flex": -10,
}


LOOK_LEFT = {
    "shoulder_pan": -35,
    "wrist_roll": -15,
}


LOOK_RIGHT = {
    "shoulder_pan": 35,
    "wrist_roll": 15,
}


CENTER = {
    "shoulder_pan": 0,
    "wrist_roll": 0,
}


WAVE_LEFT = {
    "wrist_roll": -30,
    "wrist_flex": -20,
}


WAVE_RIGHT = {
    "wrist_roll": 30,
    "wrist_flex": 5,
}


FINISH = {
    "shoulder_pan": -25,
    "shoulder_lift": -15,
    "elbow_flex": -20,
    "wrist_flex": -15,
    "wrist_roll": -20,
}


# ------------------------------------------------------------
# Robot show
# ------------------------------------------------------------

try:

    # --------------------------------------------------------
    # 1. Smoothly move to home
    # --------------------------------------------------------

    print("Moving to home position...")

    move_smooth(
        HOME,
        duration=1.8,
        steps=60,
    )

    hold(0.3)


    # --------------------------------------------------------
    # 2. Raise arm
    # --------------------------------------------------------

    print("Raising arm...")

    move_smooth(
        RAISED,
        duration=1.08,
        steps=45,
    )

    hold(0.2)


    # --------------------------------------------------------
    # 3. Look left
    # --------------------------------------------------------

    print("Looking left...")

    move_smooth(
        LOOK_LEFT,
        duration=0.78,
        steps=35,
    )

    hold(0.2)


    # --------------------------------------------------------
    # 4. Look right
    # --------------------------------------------------------

    print("Looking right...")

    move_smooth(
        LOOK_RIGHT,
        duration=1.08,
        steps=45,
    )

    hold(0.2)


    # --------------------------------------------------------
    # 5. Center
    # --------------------------------------------------------

    print("Centering...")

    move_smooth(
        CENTER,
        duration=0.72,
        steps=30,
    )


    # --------------------------------------------------------
    # 6. Wave
    # --------------------------------------------------------

    print("Waving...")

    for i in range(3):

        move_smooth(
            WAVE_LEFT,
            duration=0.33,
            steps=20,
        )

        move_smooth(
            WAVE_RIGHT,
            duration=0.33,
            steps=20,
        )


    # --------------------------------------------------------
    # 7. Open gripper
    # --------------------------------------------------------

    print("Opening gripper...")

    move_smooth(
        {
            "gripper": 80,
        },
        duration=0.48,
        steps=25,
    )

    hold(0.3)


    # --------------------------------------------------------
    # 8. Close gripper
    # --------------------------------------------------------

    print("Closing gripper...")

    move_smooth(
        {
            "gripper": 20,
        },
        duration=0.48,
        steps=25,
    )

    hold(0.3)


    # --------------------------------------------------------
    # 9. Finishing pose
    # --------------------------------------------------------

    print("Finishing pose...")

    move_smooth(
        FINISH,
        duration=0.9,
        steps=40,
    )

    hold(0.6)


    # --------------------------------------------------------
    # 10. Return home
    # --------------------------------------------------------

    print("Returning home...")

    move_smooth(
        HOME,
        duration=1.5,
        steps=60,
    )

    print("Demo complete!")


finally:

    # Disable torque
    for motor in motors:
        bus.write(
            "Torque_Enable",
            motor,
            0,
        )

    bus.disconnect()