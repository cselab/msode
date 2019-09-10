

all: main


main:
	make -C core/
	make -C rl/

clean:
	make -C core/ clean
	make -C rl/   clean

.PHONY: all clean
