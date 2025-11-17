SRC = main.c
PI_INC = rs02.c pigpio.c
JET_INC = rs02.c jetgpio.c

PI_OBJ = $(PI_INC:%.c=%.o)
JET_OBJ = $(JET_INC:%.c=%.o)
MAIN_OBJ = $(SRC:%.c=%.o)

#compile recipe
%.o: %.c 
	gcc -c $< -o $@

#pi recipe
pi_main: main.o $(PI_OBJ) 
	gcc $^ -o $@ 

#jetson recipe
jet_main: main.o $(JET_OBJ) 
	gcc $^ -o $@