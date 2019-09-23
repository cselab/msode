include ../config.mk

CORE_DIR=../core

CXXFLAGS += -I$(CORE_DIR)
LDFLAGS += -L$(CORE_DIR) -lmsode

EXECS = rotating forward forward_curve

all: $(EXECS)

%.o: %.cpp; $(LD) $(CXXFLAGS) -c $< -o $@

rotating: rotating.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
forward: forward.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
forward_curve: forward_curve.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)


.PHONY: all clean
clean:; rm -rf $(EXECS) *.o
