include ../config.mk

CORE_DIR  = ../core
EIGEN_DIR = ../extern/eigen-git-mirror

CXXFLAGS += -I$(CORE_DIR)
CXXFLAGS += -I$(EIGEN_DIR)

KORALICFLAGS = `python3 -m korali.cxx --cflags`
KORALILIBS   = `python3 -m korali.cxx --libs`

CXX_SMARTIES = -I${SMARTIES_ROOT}/include
LD_SMARTIES  = -L${SMARTIES_ROOT}/lib -lsmarties

LDFLAGS += -L$(CORE_DIR) -lmsode

EXECS = app_rotating app_forward app_forward_curve app_analytic_control app_orient app_rl

all: $(EXECS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

app_rotating:      app_rotating.o;      $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
app_orient:        app_orient.o;        $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
app_forward:       app_forward.o;       $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
app_forward_curve: app_forward_curve.o; $(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

app_analytic_control.o: app_analytic_control.cpp analytic_control/helpers.h analytic_control/optimal_path.h
	$(CXX) $(CXXFLAGS) $(KORALICFLAGS) -c $< -o $@

analytic_control/helpers.o: analytic_control/helpers.cpp analytic_control/helpers.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

analytic_control/optimal_path.o: analytic_control/optimal_path.cpp analytic_control/optimal_path.h analytic_control/helpers.h
	$(CXX) $(CXXFLAGS) $(KORALICFLAGS) -c $< -o $@

app_analytic_control: app_analytic_control.o analytic_control/helpers.o analytic_control/optimal_path.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(KORALILIBS)


app_rl.o: app_rl.cpp rl/environment.h rl/magnetic_field_from_action.h
	$(CXX) $(CXXFLAGS) $(CXX_SMARTIES) -c $< -o $@

app_rl: app_rl.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LD_SMARTIES)

.PHONY: all clean
clean:; rm -rf $(EXECS) *.o
