import time
import math

from lidar_lite import read_distance
from motion import set_wheel_speeds
from odometry import get_odometry


SAFE_DISTANCE = 90
STOP_DISTANCE = 50
CLEAR_DISTANCE = 130
FORWARD_SPEED = 0.4
TURN_GAIN = 0.015
ESCAPE_TIME = 1.2

state = "FORWARD"
turn_direction = 1
turn_start_time = 0


def forward_with_steering(turn):

    fl = FORWARD_SPEED - turn
    fr = FORWARD_SPEED + turn
    bl = FORWARD_SPEED - turn
    br = FORWARD_SPEED + turn

    return fl, fr, bl, br


def rotate(turn):

    return (
        -turn,
        turn,
        -turn,
        turn
    )


def reverse():

    s = -0.3
    return (s, s, s, s)


while True:

    d = read_distance()

    x, y, theta = get_odometry()

    if d is None:
        continue

    now = time.time()

    print("distance:", d, "state:", state)


    # --------------------------
    # NORMAL FORWARD NAVIGATION
    # --------------------------

    if state == "FORWARD":

        if d < STOP_DISTANCE:

            state = "TURNING"

            turn_direction = 1 if (time.time() % 2) > 1 else -1

            turn_start_time = now

            continue


        if d < SAFE_DISTANCE:

            # virtual corridor steering
            avoidance_strength = (SAFE_DISTANCE - d)

            turn = TURN_GAIN * avoidance_strength

            turn *= turn_direction

        else:

            turn = 0


        set_wheel_speeds(*forward_with_steering(turn))


    # --------------------------
    # HARD TURNING
    # --------------------------

    elif state == "TURNING":

        set_wheel_speeds(*rotate(0.35 * turn_direction))

        if d > CLEAR_DISTANCE:

            state = "FORWARD"


        elif now - turn_start_time > 2.0:

            state = "ESCAPE"

            escape_start = now


    # --------------------------
    # WALL ESCAPE
    # --------------------------

    elif state == "ESCAPE":

        elapsed = now - escape_start

        if elapsed < ESCAPE_TIME:

            set_wheel_speeds(*reverse())

        elif elapsed < ESCAPE_TIME + 1.0:

            set_wheel_speeds(*rotate(0.5 * turn_direction))

        else:

            state = "FORWARD"


    time.sleep(0.05)