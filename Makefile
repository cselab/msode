CXX=gcc
LD=g++
CXXFLAGS=-O3 -Wpedantic -std=c++14 -Wall

debug ?= 0

ifeq ($(debug),1)
CXXFLAGS += -DFAIL_ON_CONTRACT
endif

SRC=src

.PHONY: clean

all: main

factory.o: $(SRC)/factory.cpp $(SRC)/factory.h $(SRC)/simulation.h $(SRC)/quaternion.h $(SRC)/types.h $(SRC)/math.h $(SRC)/log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main.o: $(SRC)/main.cpp $(SRC)/simulation.h $(SRC)/factory.h  $(SRC)/quaternion.h $(SRC)/types.h $(SRC)/log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

simulation.o: $(SRC)/simulation.cpp $(SRC)/simulation.h $(SRC)/quaternion.h $(SRC)/types.h $(SRC)/math.h $(SRC)/log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main: main.o simulation.o factory.o
	$(LD) $(CXXFLAGS) -o $@ $^

clean:; rm -rf main *.o
