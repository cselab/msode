include ../config.mk

CORE_DIR=../core

CXXFLAGS += -I$(CORE_DIR)
LDFLAGS += -L$(CORE_DIR) -lmsode

all: main

environment.o: environment.cpp environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main.o: main.cpp  environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main: main.o environment.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: all clean
clean: rm main
