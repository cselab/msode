include ../config.mk

CORE_DIR=../core

CXXFLAGS += -I$(CORE_DIR)
LDFLAGS += -L$(CORE_DIR) -lmsode

CXXFLAGS += -Wall -O2 -I${SMARTIES_ROOT}/include
LDFLAGS  += -L${SMARTIES_ROOT}/lib -lsmarties

all: main test_environment

environment.o: environment.cpp environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main.o: main.cpp  environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

test_environment.o: test_environment.cpp  environment.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main: main.o environment.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)


test_environment: test_environment.o environment.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: all clean
clean:; rm -rf main *.o
