import smbus2
import time

LIDAR_ADDR = 0x62
ACQ_COMMAND = 0x00
DIST_HIGH = 0x0f
DIST_LOW = 0x10

bus = smbus2.SMBus(1)

def read_distance():

    try:
        bus.write_byte_data(LIDAR_ADDR, ACQ_COMMAND, 0x04)
        time.sleep(0.02)

        high = bus.read_byte_data(LIDAR_ADDR, DIST_HIGH)
        low = bus.read_byte_data(LIDAR_ADDR, DIST_LOW)

        return (high << 8) + low

    except:
        return None
