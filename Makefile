CORE_DIR=src/msode
APPS_DIR=apps

all: apps

libmsode.a:
	@$(MAKE) -C $(CORE_DIR) $@

apps: libmsode.a
	@$(MAKE) -C $(APPS_DIR) all

clean:
	$(MAKE) -C $(CORE_DIR) clean
	$(MAKE) -C $(APPS_DIR) clean

.PHONY: all clean apps
