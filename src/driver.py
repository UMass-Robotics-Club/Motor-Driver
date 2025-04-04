import threading
import struct
from sys import stdin, stdout
import time

###############
# User Config #
###############

class MotorConfig:
    def __init__(self, motor_type: bool, channel, id):
        self.motor_type = motor_type
        self.channel = channel
        self.id = id

# Config in the form
# (Type, CAN Channel, ID)
# Type:
#   SPARK MAX: 0
#   AK60: 1
# CAN Channel: 0-5
# ID: 0-63
MOTOR_CONFIGS = [
    MotorConfig(False, 0, 0),
    MotorConfig(False, 0, 0),
    MotorConfig(True, 0, 0),
    MotorConfig(True, 0, 0),
    MotorConfig(False, 0, 0),
    MotorConfig(False, 0, 0),
]
TIMEOUT = 5


#############
# Constants #
#############
AK60_POSITION_SETPOINT = 0x400
AK60_POWER_SETPOINT = 0x00

SPARK_POSITION_SETPOINT = 0x2050100
ROBORIO_HEARTBEAT = 0x01011840


########
# Code #
########

positions = [ 0, 0, 0, 0, 0, 0 ]
last_update_time = time.time()
position_lock = threading.Lock()

def gen_can_header(channel: int, data: int, arbitration: int) -> bytes:
    return (channel + (data << 3)).to_bytes() + arbitration.to_bytes(4, "little", signed=False)

def gen_position_spark_packet(channel: int, id: int, degree: float):
    return  gen_can_header(channel, 8, ROBORIO_HEARTBEAT) + (b'\xff' * 8) + \
            gen_can_header(channel, 8, SPARK_POSITION_SETPOINT + id) + struct.pack("<f", degree / 360) + (b'\x00' * 4)

def gen_position_ak60_packet(channel: int, id: int, degree: float):
    return gen_can_header(channel, 4, AK60_POSITION_SETPOINT + id) + struct.pack(">i", degree * 10000)

def gen_power_ak60_packet(channel: int, id: int, power: float):
    return gen_can_header(channel, 4, AK60_POWER_SETPOINT + id) + struct.pack(">i", power * 100000)

def handle_user_input():
    global positions
    global last_update_time

    while(1):
        data = stdin.buffer.read(24)
        position_lock.acquire()
        positions = struct.unpack("<6f", data)
        last_update_time = time.time()
        position_lock.release()

def emergency_shutdown():
    for config in MOTOR_CONFIGS:
        if(config.motor_type):
            stdout.buffer.write(gen_power_ak60_packet(config.channel, config.id, 0))

def main():
    t = threading.Thread(target=handle_user_input)
    t.start()

    while(1):
        position_lock.acquire()
        if time.time() - last_update_time > TIMEOUT:
            emergency_shutdown()
            exit(1)
        for idx, config in enumerate(MOTOR_CONFIGS):
            if(config.motor_type):
                # AK60
                stdout.buffer.write(gen_position_ak60_packet(config.channel, config.id, positions[idx]))
            else:
                # Spark MAX
                stdout.buffer.write(gen_position_spark_packet(config.channel, config.id, positions[idx]))

if __name__ == "__main__":
    main()