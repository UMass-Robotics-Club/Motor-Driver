import threading
import struct
from sys import stdin
import time
import spidev

from utils import gen_position_spark_packet, gen_position_ak60_packet, gen_power_ak60_packet

###############
# User Config #
###############

# Config in the form
# (Type, CAN Channel, ID)
# Type:
#   SPARK MAX: False
#   AK60: True
# CAN Channel: 0-5
# ID: 0-63
class MotorConfig:
    def __init__(self, motor_type: bool, channel, id):
        self.motor_type = motor_type
        self.channel = channel
        self.id = id

MOTOR_CONFIGS = [
    MotorConfig(False, 0, 0),
    MotorConfig(False, 0, 0),
    MotorConfig(True, 0, 0),
    MotorConfig(True, 0, 0),
    MotorConfig(False, 0, 0),
    MotorConfig(False, 0, 0),
]

TIMEOUT = 5
SPI_BUS = 0
SPI_DEV = 0
SPI_SPEED = 1000000 # 1 MHz


########
# Code #
########

positions = [ 0, 0, 0, 0, 0, 0 ]
last_update_time = time.time()
position_lock = threading.Lock()


def handle_user_input():
    global positions
    global last_update_time

    while(1):
        data = stdin.buffer.read(24)
        position_lock.acquire()
        positions = struct.unpack("<6f", data)
        last_update_time = time.time()
        position_lock.release()
        print(f"[DEBUG] Recived positions {positions}")

def emergency_shutdown(spi):
    for config in MOTOR_CONFIGS:
        if(config.motor_type):
            spi.write(gen_power_ak60_packet(config.channel, config.id, 0))

def main():
    print("[INFO] Connecting to device...")
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEV)
    spi.max_speed_hz = SPI_SPEED

    print("[INFO] Starting user input thread")
    t = threading.Thread(target=handle_user_input)
    t.start()

    while(1):
        position_lock.acquire()
        if time.time() - last_update_time > TIMEOUT:
            print(f"[ERROR] No data within {TIMEOUT} seconds, starting emergency shutdown.")
            emergency_shutdown()
            spi.close()
            exit(1)
        pos_cpy = positions.copy()
        position_lock.release()

        for idx, config in enumerate(MOTOR_CONFIGS):
            if(config.motor_type):
                # AK60
                spi.writebytes(gen_position_ak60_packet(config.channel, config.id, pos_cpy[idx]))
            else:
                # Spark MAX
                spi.writebytes(gen_position_spark_packet(config.channel, config.id, pos_cpy[idx]))

if __name__ == "__main__":
    main()
