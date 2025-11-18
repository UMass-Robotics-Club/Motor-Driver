SRC = main.c
PI_INC = rs02.c pigpio.c command.c custom.cext
JET_INC = rs02.c jetgpio.c

PI_OBJ = $(PI_INC:%.c=%.o) $(PI_INC:%.cext=%.o)
JET_OBJ = $(JET_INC:%.c=%.o)
MAIN_OBJ = $(SRC:%.c=%.o)

#compile recipes
%.o: %.c 
	gcc -c $< -o $@

%.o: %.cext
	gcc -c $< -o $@

#pi recipe
pi_main: main.o $(PI_OBJ) 
	gcc $^ -o $@ 

#jetson recipe
jet_main: main.o $(JET_OBJ) 
	gcc $^ -o $@