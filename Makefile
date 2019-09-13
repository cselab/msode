CORE_DIR=core
APPS_DIR=apps
RL_DIR=rl

all: msode_rl apps

libmsode.a:
	@make -C $(CORE_DIR) $@

apps:
	@make -C $(APPS_DIR) all

msode_rl: libmsode.a
	@make -C $(RL_DIR) main
	@cp $(RL_DIR)/main $@

clean:
	make -C $(CORE_DIR) clean
	make -C $(APPS_DIR) clean
	make -C $(RL_DIR)   clean
	rm -rf msode_rl

.PHONY: all clean apps
