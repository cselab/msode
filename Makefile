CXX=gcc
CXXFLAGS=-O3 -Wpedantic -std=c++14 -Wall

SRC=src

.PHONY: clean

all: main

main.o: $(SRC)/main.cpp $(SRC)/simulation.h  $(SRC)/quaternion.h $(SRC)/types.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

simulation.o: $(SRC)/simulation.cpp $(SRC)/simulation.h $(SRC)/quaternion.h $(SRC)/types.h $(SRC)/math.h 
	$(CXX) $(CXXFLAGS) -c $< -o $@

main: main.o simulation.o
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:; rm -rf main *.o
