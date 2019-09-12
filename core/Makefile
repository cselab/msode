include ../config.mk

.PHONY: clean all

all: main libmsode.a

file_parser.o: file_parser.cpp file_parser.h log.h simulation.h types.h quaternion.h math.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

factory.o: factory.cpp factory.h simulation.h quaternion.h types.h math.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

main.o: main.cpp simulation.h factory.h  quaternion.h types.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

simulation.o: simulation.cpp simulation.h quaternion.h types.h math.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@



main: main.o simulation.o factory.o file_parser.o
	$(LD) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

libmsode.a: factory.o simulation.o file_parser.o
	ar rcs $@ $^



clean:; rm -rf main *.o *.a
