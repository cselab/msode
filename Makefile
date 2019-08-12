CXX=gcc
CXXFLAGS=-O3 -Wpedantic -std=c++14 -Wall

SRC=src

.PHONY: clean

all: main

main.o: $(SRC)/main.cpp $(SRC)/quaternion.h $(SRC)/types.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main: main.o
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:; rm -rf main *.o
