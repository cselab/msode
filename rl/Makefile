include ../config.mk

CORE_DIR=../core

CXXFLAGS += -I$(CORE_DIR)
LDFLAGS += -L$(CORE_DIR) -lmsode

CXXFLAGS += -Wall -O2 -I${SMARTIES_ROOT}/include
LDFLAGS  += -L${SMARTIES_ROOT}/lib -lsmarties

all: msode_rl

environment.o: environment.cpp environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main.o: main.cpp  environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

msode_rl: main.o environment.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: all clean
clean:; rm -rf msode_rl *.o
