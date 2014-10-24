# changelog :
#------------
# naresh - 29-Mar-2007 : cyan_00391 
# 	fixed the code which detects the existing kernel module
#
# Notes:
# -----------
# This clever makefile was shamelessly copied from the ivtv project.
#
# By default, the build is done against the running kernel version.
# to build against a different kernel version, set KVER
#
#  make KVER=2.6.11-alpha
#
#  Alternatively, set KDIR
#
#  make KDIR=/usr/src/linux

V ?= 0
MDIR := extra

KVER ?= $(shell uname -r)
KDIR ?= /lib/modules/$(KVER)/build
FWDIR ?= /lib/firmware

# Old module name to detect and complain about when installing
OLD_MODULE_NM = ry5u870.ko

FWFILES = r5u870_1830.fw r5u870_1832.fw r5u870_1833.fw r5u870_1834.fw r5u870_1835.fw r5u870_1836.fw r5u870_1870_1.fw r5u870_1870.fw r5u870_1810.fw r5u870_183a.fw r5u870_183b.fw r5u870_1839.fw r5u870_1841.fw

ifneq ($(KERNELRELEASE),)
include $(src)/Kbuild
else

all::
	$(MAKE) -C $(KDIR) M=$(CURDIR) V=$(V) modules

install:: all
	$(MAKE) INSTALL_MOD_PATH=$(DESTDIR) INSTALL_MOD_DIR=$(MDIR) \
		-C $(KDIR) M=$(CURDIR) modules_install

clean::
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean
	rm -f Module.symvers

endif

install::
	install -m 0644 -o root -g root $(FWFILES) $(FWDIR)
	/sbin/depmod -a
	@if find /lib/modules -name $(OLD_MODULE_NM) | grep $(OLD_MODULE_NM) >/dev/null; then \
		echo; \
		echo "*** !!! WARNING !!! ***"; \
		echo "A prior version of this driver was detected, installed with a different file"; \
		echo "name.  It is possible that a boot-time device detection component will choose"; \
		echo "to load the older version of this driver instead of the newly installed"; \
		echo "version."; \
		echo; \
		echo "Please consider deleting the following files:"; \
		echo; \
		find /lib/modules -name $(OLD_MODULE_NM); \
		echo; \
		echo "*** !!! WARNING !!! ***"; \
		echo; \
		printf "\\a"; sleep 1; printf "\\a"; sleep 1; printf "\\a"; \
	fi
