# ARM Cortex-Mx common makefile scripts and rules.

##############################################################################
# Processing options coming from the upper Makefile.
#

# Output directory and files
SRECFILE = $(BUILDDIR)/$(PROJECT).srec

#
# Makefile rules
#

blt: $(SRECFILE)

MAKE_ALL_RULE_HOOK: blt

%.srec: %.elf $(LDSCRIPT)
ifeq ($(USE_VERBOSE_COMPILE),yes)
	$(SREC) $< $@
else
	@echo Creating $@
	@$(SREC) $< $@
endif

# *** EOF ***
