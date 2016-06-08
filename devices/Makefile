all: devices

include flash.mk

CLEANTRGS = $(DEVICES:%=clean-%)

.PHONY: $(DEVICES)

devices: $(DEVICES)

$(DEVICES):
	$(MAKE) -C $@

$(FLASHTRGS):
	$(MAKE) -C $(firstword $(subst /, ,$@))

clean: $(CLEANTRGS)
$(CLEANTRGS):
	$(MAKE) -C $(@:clean-%=%) clean
