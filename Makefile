CORE_DIR=core
RL_DIR=rl

all: msode_rl msode test_environment

msode:
	@make -C $(CORE_DIR) main
	@cp $(CORE_DIR)/main $@

libmsode.a:
	@make -C $(CORE_DIR) $@

msode_rl: libmsode.a
	@make -C $(RL_DIR) main
	@cp $(RL_DIR)/main $@

test_environment: libmsode.a
	@make -C $(RL_DIR) test_environment
	@cp $(RL_DIR)/test_environment $@

clean:
	make -C $(CORE_DIR) clean
	make -C $(RL_DIR)   clean
	rm -rf msode msode_rl test_environment

.PHONY: all clean
