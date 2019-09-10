CORE_DIR=core
RL_DIR=rl

all: mainRl mainTest

mainTest:
	@make -C $(CORE_DIR) main
	@cp $(CORE_DIR)/main $@

libmsode.a:
	@make -C $(CORE_DIR) $@

mainRl: libmsode.a
	@make -C $(RL_DIR)
	@cp $(RL_DIR)/main $@

clean:
	make -C $(CORE_DIR) clean
	make -C $(RL_DIR)   clean

.PHONY: all clean
