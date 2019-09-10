include ../config.mk

.PHONY: clean all

all: main libmsode.a

factory.o: factory.cpp factory.h simulation.h quaternion.h types.h math.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main.o: main.cpp simulation.h factory.h  quaternion.h types.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

simulation.o: simulation.cpp simulation.h quaternion.h types.h math.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@



main: main.o simulation.o factory.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

libmsode.a: factory.o simulation.o
	ar rcs $@ $^



clean:; rm -rf main *.o *.a
