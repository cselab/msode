include ../config.mk

CORE_DIR=../core
CATCH_DIR = ../extern/Catch2

CXXFLAGS += -I$(CORE_DIR)
CXXFLAGS += -I$(CATCH_DIR)/single_include/catch2

LDFLAGS += -L$(CORE_DIR) -lmsode

EXECS = test_quaternions

all: $(EXECS)

%.o: %.cpp; $(CXX) $(CXXFLAGS) -c $< -o $@

test_quaternions: test_quaternions.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

.PHONY: all clean
clean:; rm -rf $(EXECS) *.o
