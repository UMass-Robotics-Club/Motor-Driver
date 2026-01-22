PI4_SRC = main_pi4.c
PI5_SRC = main.pi5.c
PI_INC = rs02.c 
JET_INC = rs02.c jetgpio.c


PI5_LIB = -llgpio -lpthread -lrt
PI5_OBJ = $(PI_INC:%.c=%.o)

PI4_LIB = -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt
PI4_OBJ = $(PI_INC:%.c:%.o)


JET_OBJ = $(JET_INC:%.c=%.o)
PI4_MAIN_OBJ = $(PI4_SRC:%.c=%.o)
PI5_MAIN_OBJ = $(PI5_SRC:%.c=%.o)

#compile recipes
%.o: %.c 
	gcc -c $< -o $@


#pi4 recipe
pi4_main: main_pi4.o $(PI4_OBJ)
	gcc $^ $(PI4_LIB) -o $@

#pi5 recipe
pi5_main: main_pi5.o $(PI5_OBJ) 
	gcc $^ $(PI5_LIB) -o $@ 

#jetson recipe
jet_main: main.o $(JET_OBJ) 
	gcc $^ -o $@