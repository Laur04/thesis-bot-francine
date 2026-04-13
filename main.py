import smbus
import struct
import time

from lidar_lite import read_distance


## CONSTANTS
# navigation
DEFAULT_CRUISE_SPEED = 80
DEFAULT_TURN_SPEED = 30
STOP_DISTANCE = 30
REORIENT_TIME = 10
FULL_TURN_ODOMETRY = 5600

# motor control
MOTOR_TYPE = 1
MOTOR_MODEL_ADDR = 0x26
MOTOR_TYPE_REG = 0x01
MOTOR_DEADZONE_REG = 0x02
MOTOR_PLUSELINE_REG = 0x03
MOTOR_PLUSEPHASE_REG = 0x04
WHEEL_DIA_REG = 0x05
SPEED_CONTROL_REG = 0x06
PWM_CONTROL_REG = 0x07
# READ_ALLHIGH_M1_REG = 0x20
# READ_ALLLOW_M1_REG = 0x21
# READ_ALLHIGH_M2_REG = 0x22
# READ_ALLLOW_M2_REG = 0x23
# READ_ALLHIGH_M3_REG = 0x24
# READ_ALLLOW_M3_REG = 0x25
# READ_ALLHIGH_M4_REG = 0x26
# READ_ALLLOW_M4_REG = 0x27
READ_ALLHIGH_M1_REG = 0x22
READ_ALLLOW_M1_REG = 0x23
READ_ALLHIGH_M2_REG = 0x26
READ_ALLLOW_M2_REG = 0x27
READ_ALLHIGH_M3_REG = 0x24
READ_ALLLOW_M3_REG = 0x25
READ_ALLHIGH_M4_REG = 0x20
READ_ALLLOW_M4_REG = 0x21

# encoder
# READ_TEN_M1_ENCODER_REG = 0x10
# READ_TEN_M2_ENCODER_REG = 0x11
# READ_TEN_M3_ENCODER_REG = 0x12
# READ_TEN_M4_ENCODER_REG = 0x13
READ_TEN_M1_ENCODER_REG = 0x11
READ_TEN_M2_ENCODER_REG = 0x13
READ_TEN_M3_ENCODER_REG = 0x12
READ_TEN_M4_ENCODER_REG = 0x10


## GLOBALS
# navigation
encoder_now = [0] * 4

# I2C
bus = smbus.SMBus(1)


## FUNCTIONS
# utility
def float_to_bytes(f):
    return struct.pack('<f', f)

# I2C
def i2c_write(addr, reg, data):
    bus.write_i2c_block_data(addr, reg, data)

def i2c_read(addr, reg, length):
    return bus.read_i2c_block_data(addr, reg, length)

# motor control
def set_motor_type(data):
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_TYPE_REG, [data])

def set_motor_deadzone(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_DEADZONE_REG, buf)

def set_pluse_line(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSELINE_REG, buf)

def set_pluse_phase(data):
    buf = [(data >> 8) & 0xFF, data & 0xFF]
    i2c_write(MOTOR_MODEL_ADDR, MOTOR_PLUSEPHASE_REG, buf)

def set_wheel_dis(data):
    bytes_data = float_to_bytes(data)
    i2c_write(MOTOR_MODEL_ADDR, WHEEL_DIA_REG, list(bytes_data))

def control_speed(m1, m2, m3, m4):
    # speeds = [
    #     (m1 >> 8) & 0xFF, m1 & 0xFF,
    #     (m2 >> 8) & 0xFF, m2 & 0xFF,
    #     (m3 >> 8) & 0xFF, m3 & 0xFF,
    #     (m4 >> 8) & 0xFF, m4 & 0xFF
    # ]
    speeds = [
        (m4 >> 8) & 0xFF, m4 & 0xFF,
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF        
    ]
    print(f"setting speed {m1} {m2} {m3} {m4}")
    i2c_write(MOTOR_MODEL_ADDR, SPEED_CONTROL_REG, speeds)

def control_pwm(m1, m2, m3, m4):
    # pwms = [
    #     (m1 >> 8) & 0xFF, m1 & 0xFF,
    #     (m2 >> 8) & 0xFF, m2 & 0xFF,
    #     (m3 >> 8) & 0xFF, m3 & 0xFF,
    #     (m4 >> 8) & 0xFF, m4 & 0xFF
    # ]
    pwms = [
        (m4 >> 8) & 0xFF, m4 & 0xFF,
        (m1 >> 8) & 0xFF, m1 & 0xFF,
        (m3 >> 8) & 0xFF, m3 & 0xFF,
        (m2 >> 8) & 0xFF, m2 & 0xFF
    ]
    i2c_write(MOTOR_MODEL_ADDR, PWM_CONTROL_REG, pwms)

# navigation
def straight(speed=DEFAULT_CRUISE_SPEED):
    fl = fr = speed
    br = bl = -speed
    return fl, fr, bl, br

def rotate(speed=DEFAULT_TURN_SPEED):
    fl = br = speed
    bl = fr = -speed
    return fl, fr, bl, br

def read_all_encoder():
    global encoder_now
    for i in range(4):
        high_reg = READ_ALLHIGH_M1_REG + (i * 2)
        low_reg = READ_ALLLOW_M1_REG + (i * 2)
        high_buf = i2c_read(MOTOR_MODEL_ADDR, high_reg, 2)
        low_buf = i2c_read(MOTOR_MODEL_ADDR, low_reg, 2)
        high_val = high_buf[0] <<8 | high_buf[1]
        low_val = low_buf[0] <<8 | low_buf[1]
        encoder_val = (high_val << 16) | low_val
        
        if encoder_val >= 0x80000000:
            encoder_val -= 0x100000000
        encoder_now[i] = encoder_val

def retrieve_sound_theta():
    # data = spi.xfer2([0, 0, 0, 0])
    # if data[0] == 0xAA:
    #     theta = (data[1] << 16) | (data[3]) << 8 | data[3]
    #     return theta * 360 / (2 ** 23) 
    print("[ERROR] failed to retrieve sound data properly")
    return 0

def orient():
    # stop moving and take a reading
    control_pwm(0, 0, 0, 0)
    time.sleep(1)
    theta_to_turn = retrieve_sound_theta()
    odometry_to_turn = (FULL_TURN_ODOMETRY / 360) * theta_to_turn

    while 10 < abs(odometry_to_turn) > 350:
        # perform turn
        read_all_encoder()
        initial = [x for x in encoder_now]
        while not all(a >= odometry_to_turn for a in [abs(x - y) for x, y in zip(encoder_now, initial)]):
            control_speed(*rotate())
            read_all_encoder()
        control_pwm(0, 0, 0, 0)

        # stop moving and take a reading
        time.sleep(1)
        theta_to_turn = retrieve_sound_theta()
        odometry_to_turn = (FULL_TURN_ODOMETRY / 360) * theta_to_turn
    
    return time.time()


## MAIN
if __name__ == "__main__":

    # Motor initialization
    set_motor_type(1)
    time.sleep(0.1)
    set_pluse_phase(30)
    time.sleep(0.1)
    set_pluse_line(11)
    time.sleep(0.1)
    set_wheel_dis(67.00)
    time.sleep(0.1)
    set_motor_deadzone(1600)

    # Main control loop
    try:
        last_orient_time = orient()
        while True:

            # Reorient to the sound every REORIENT_TIME seconds
            if time.time() - last_orient_time > REORIENT_TIME:
                last_orient_time = orient()

            # Check if destination has been reached

            # Obstacle avoidance
            if read_distance() < STOP_DISTANCE:
                while read_distance() < STOP_DISTANCE:
                    control_speed(*rotate())
            
            # Straight travel
            print("going straight")
            control_speed(*straight())

            time.sleep(0.05)

    # To stop all motor activity
    except KeyboardInterrupt:
        control_pwm(0, 0, 0, 0)
