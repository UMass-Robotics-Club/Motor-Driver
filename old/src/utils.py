import struct

#############
# Constants #
#############

AK60_POSITION_SETPOINT = 0x400
AK60_POWER_SETPOINT = 0x00

SPARK_POSITION_SETPOINT = 0x2050100
ROBORIO_HEARTBEAT = 0x01011840



def gen_can_header(channel: int, data: int, arbitration: int) -> bytes:
    return (channel + (data << 3)).to_bytes() + arbitration.to_bytes(4, "little", signed=False)

def gen_position_spark_packet(channel: int, id: int, degree: float) -> bytes:
    return  gen_can_header(channel, 8, ROBORIO_HEARTBEAT) + (b'\xff' * 8) + \
            gen_can_header(channel, 8, SPARK_POSITION_SETPOINT + id) + struct.pack("<f", degree / 360) + (b'\x00' * 4)

def gen_position_ak60_packet(channel: int, id: int, degree: float) -> bytes:
    return gen_can_header(channel, 4, AK60_POSITION_SETPOINT + id) + struct.pack(">i", degree * 10000)

def gen_power_ak60_packet(channel: int, id: int, power: float) -> bytes:
    return gen_can_header(channel, 4, AK60_POWER_SETPOINT + id) + struct.pack(">i", power * 100000)