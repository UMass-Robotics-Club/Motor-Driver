import serial
import time
import random
import struct

from utils import gen_position_spark_packet, gen_can_header, gen_rs02_mit, RS02_ENABLE, RS02_MIT_MODE, RS02_MIT_POS_MODE

PORT = "COM3"
BAUD = 115200
TIMEOUT = 3

conn = serial.Serial(PORT, BAUD)


def send_alternating_spark_packets() -> None:
    last_switch = 0
    dir = True
    while(1):
        print(f"Sending {"forward" if dir else "backwards"} packet")
        conn.write(gen_position_spark_packet(0, 1, 360*(10 if dir else -10)))
        if time.time() > last_switch + TIMEOUT:
            last_switch = time.time()
            dir = not dir



def flood_can_controller() -> None:
    while(1):
        time.sleep(1)
        controller = 1
        db = b"deadbeef"
        print(f"Sending packet to controller {controller}")
        conn.write(gen_can_header(controller, 8, 0x0)+db)
        for _ in range(4):
            print(conn.readline().decode(), end="")


def rs02_setup(channel: int):
    print(f"Setting up position mode on controller {channel}")
    conn.write(gen_rs02_mit(channel, 0, RS02_ENABLE))
    print("Sent ENABLE packet")
    time.sleep(2)
    conn.write(gen_rs02_mit(channel, 0xFFF, RS02_MIT_MODE))
    print("Sent MIT MODE packet")
    time.sleep(2)
    conn.write(gen_rs02_mit(channel, 0, RS02_MIT_POS_MODE))
    print("Sent MIT POSITION MODE packet \n Setup Complete \n")


def rs02_main_test_can0(channel: int):
    sp = struct.pack("<f", 2)
    pos = struct.pack("<f", 4)
    while(1):
        print(f"rotate")
        conn.write(gen_rs02_mit(channel, 0, pos + sp))
        time.sleep(5)
        


if __name__ == "__main__":
    #kys()
    #flood_can_controller()
    rs02_setup(1)
    rs02_main_test_can0(1)