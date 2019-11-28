include ../config.mk

CXXFLAGS += -Wextra -Wshadow -Werror

all: libmsode.a

file_parser.o: file_parser.cpp file_parser.h log.h simulation.h types.h quaternion.h math.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

factory.o: factory.cpp factory.h simulation.h quaternion.h types.h math.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

simulation.o: simulation.cpp simulation.h quaternion.h types.h math.h log.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

libmsode.a: factory.o simulation.o file_parser.o
	ar rcs $@ $^

.PHONY: clean all
clean:; rm -rf *.o *.a
