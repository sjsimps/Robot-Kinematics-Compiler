
CPP_FLAGS = -std=c++11
DEBUG_FLAGS += -g -O0
PROG = robotics
TEST = robotics_test

SRC = example.cpp

INC = -I./symboliccpp/headers/

SDL =-L/usr/local/lib -lSDL2 -lSDL2_image

.PHONY: all
all: $(PROG)

.PHONY: debug
debug:
	g++-4.9 $(CPP_FLAGS) $(DEBUG_FLAGS) $(SRC) -o $(PROG)

.PHONY: clean
clean:
	rm $(PROG)
	rm $(TEST)

$(PROG):
	g++-4.9 -O2 $(INC) $(CPP_FLAGS) $(SRC) $(SDL) -o $(PROG)

debug:
	g++-4.9 -O2 $(INC) $(CPP_FLAGS) $(DEBUG_FLAGS) $(SRC) $(SDL) -o $(PROG)

test:
	g++ -O2 -I/usr/include/eigen3/ $(CPP_FLAGS) example_out.cpp -o $(TEST)
