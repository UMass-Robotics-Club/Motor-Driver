SRC = main.c
PI_INC = rs02.c 
JET_INC = rs02.c jetgpio.c


PI_LIB = -llgpio -lpthread -lrt
PI_OBJ = $(PI_INC:%.c=%.o)
JET_OBJ = $(JET_INC:%.c=%.o)
MAIN_OBJ = $(SRC:%.c=%.o)

#compile recipes
%.o: %.c 
	gcc -c $< -o $@

#pi recipe
pi_main: main.o $(PI_OBJ) 
	gcc $^ $(PI_LIB) -o $@ 

#jetson recipe
jet_main: main.o $(JET_OBJ) 
	gcc $^ -o $@