
CPP_FLAGS = -std=c++11
DEBUG_FLAGS += -g -O0
PROG = robotics

SRC = robotics.cpp 

INC = -I./symboliccpp/headers/

.PHONY: all
all: $(PROG)

.PHONY: debug
debug:
	g++-4.9 $(CPP_FLAGS) $(DEBUG_FLAGS) $(SRC) -o $(PROG)

.PHONY: clean
clean:
	rm $(PROG)

$(PROG):
	g++-4.9 -O2 $(INC) $(CPP_FLAGS) $(SRC) -o $(PROG)

test:
	g++-4.9 -O2 $(CPP_FLAGS) out.cpp -o test
