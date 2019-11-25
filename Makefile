CORE_DIR=core
APPS_DIR=apps

all: apps

libmsode.a:
	@make -C $(CORE_DIR) $@

apps: libmsode.a
	@make -C $(APPS_DIR) all

clean:
	make -C $(CORE_DIR) clean
	make -C $(APPS_DIR) clean

.PHONY: all clean apps
