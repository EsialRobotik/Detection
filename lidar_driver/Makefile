# source files.
SRC = Lidar.cpp
OBJ = $(SRC:.cpp=.o)
OUT = libLidarDriver.a
 
# include directories
INCLUDES = -I. -I/usr/local/include
 
# C++ compiler flags (-g -O2 -Wall)
CCFLAGS = -g -O2 -Wall -std=c++11
 
# compiler
CCC = g++
 
# library paths
LIBS = -L../ -L/usr/local/lib -lm
 
# compile flags
LDFLAGS = -g
 
.SUFFIXES: .cpp
 
default: $(OUT)
 
.cpp.o:
	$(CCC) $(INCLUDES) $(CCFLAGS) -c $< -o $@
 
$(OUT): $(OBJ)
	ar rcs $(OUT) $(OBJ)

clean:
	rm -f $(OBJ) $(OUT) Makefile.bak