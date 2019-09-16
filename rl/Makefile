include ../config.mk

CORE_DIR=../core

CXXFLAGS += -I$(CORE_DIR)
LDFLAGS += -L$(CORE_DIR) -lmsode

CXXFLAGS += -Wall -O2 -I${SMARTIES_ROOT}/include
LDFLAGS  += -L${SMARTIES_ROOT}/lib -lsmarties

all: msode_rl

main.o: main.cpp  environment.h magnetic_field_from_action.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

msode_rl: main.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: all clean
clean:; rm -rf msode_rl *.o
