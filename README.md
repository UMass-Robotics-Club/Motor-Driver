# Motor-Driver
A simple C program that takes a list of positions from standard in and outputs them as formatted CAN commands for our CAN board on standard out.

## Configuration
Configuration is done based on the motor definitions in [config.h](./include/config.h). NOTE: CAN channels are 0 based indexing.

## Build Instructions
1. Make a build folder inside `code`
```
mkdir build
cd build
```
2. Create the make files thorough cmake
```
cmake ..
```
3. Build the firmware using make
```
make
```
4. The output files should be inside the `build` folder