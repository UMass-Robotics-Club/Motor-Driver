# Motor-Driver
A simple python program that takes a list of positions from standard in and outputs them as formatted CAN commands for our CAN board over SPI.

# Install
1. Setup a python3 virtual env
```
python3 -m venv ./venv
source ./venv/bin/activate
```

2. Install requirements
```
pip install -r requirements.txt
```

# Position format
Positions will be read as an array of 6 little endien floats (24 bytes total). The driver will wait for all 6 positions before sending the new positions to the motors. If it does not recive new positions for more than TIMEOUT seconds, it will enter emergency shutdown where it will turn set all motors to 0 power then exit.
