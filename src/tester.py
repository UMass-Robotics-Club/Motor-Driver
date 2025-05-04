import serial
import time

from utils import gen_position_spark_packet

PORT = "/dev/ttyACM0"
BAUD = 115200
TIMEOUT = 3

conn = serial.Serial(PORT, BAUD)

def read_n_lines(c: serial.Serial, lines: int) -> list[bytes]:
    out = []
    for _ in range(lines):
        out.append(c.readline())
    return out

last_switch = 0
dir = True
while(1):
    print(f"Sending {"forward" if dir else "backwards"} packet")
    conn.write(gen_position_spark_packet(0, 1, 360*(10 if dir else -10)))
    if time.time() > last_switch + TIMEOUT:
        last_switch = time.time()
        dir = not dir