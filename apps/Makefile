include ../config.mk

CORE_DIR=../core
EIGEN_DIR = ../extern/eigen-git-mirror

CXXFLAGS += -I$(CORE_DIR)
CXXFLAGS += -I$(EIGEN_DIR)

KORALICFLAGS=`python3 -m korali.cxx --cflags`
KORALILIBS=`python3 -m korali.cxx --libs`

LDFLAGS += -L$(CORE_DIR) -lmsode

EXECS = rotating forward forward_curve analytic_control

all: $(EXECS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

rotating: rotating.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
forward: forward.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
forward_curve: forward_curve.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

analytic_control.o: analytic_control.cpp
	$(CXX) $(CXXFLAGS) $(KORALICFLAGS) -c $< -o $@

analytic_control: analytic_control.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(KORALILIBS)

.PHONY: all clean
clean:; rm -rf $(EXECS) *.o
