import serial
import time
import random

from utils import gen_position_spark_packet, gen_can_header

PORT = "/dev/ttyACM0"
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
        controller = random.randint(0, 5)
        print(f"Sending packet to controller {controller}")
        conn.write(gen_can_header(controller, 0x0, 0x0))
        for _ in range(4):
            print(conn.readline().decode(), end="")

if __name__ == "__main__":
    flood_can_controller()