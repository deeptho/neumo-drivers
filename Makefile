# --- Variables ---
# Use ?= to allow environment overrides

KVER ?= $(shell uname -r)
#KVER := 6.17.13-200.fc42.x86_64
#KVER := 6.14.4-100.fc40.x86_64
#KVER := 6.15.3-200.fc42.x86_64
KDIR	?= /lib/modules/$(KVER)/build
PWD	:= $(shell pwd)
MDIR	?=

ROOT_DIR :=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
NEUMO_DIR :=$(ROOT_DIR)/neumo
# does not work export KBUILD_EXTMOD_OUTPUT = /tmp/build


# --- Module Configurations ---
MODDEFS := CONFIG_DVB_MMAP=y \
	CONFIG_DVB_CORE=m \
	CONFIG_DVB_TBSECP3=m \
	CONFIG_TBS_PCIE_CI=m \
	CONFIG_TBS_PCIE_MOD=m \
	CONFIG_DVB_USB_EM28XX=m \
	CONFIG_DVB_USB_CX231XX=m \
	CONFIG_DVB_SAA716X_CORE=m \
	CONFIG_DVB_SAA716X_TBS=m \
	CONFIG_DVB_SAA716X_HYBRID=m  \
	CONFIG_DVB_SI2168=m \
	CONFIG_DVB_TAS2101=m \
	CONFIG_DVB_GX1133=m \
	CONFIG_DVB_SI2183=m \
	CONFIG_DVB_CXD2878=m \
	CONFIG_DVB_STV090x=m \
	CONFIG_DVB_STV6110x=m \
	CONFIG_DVB_STV091X=m \
	CONFIG_DVB_STV0299=m \
	CONFIG_DVB_STB6000=m  \
	CONFIG_DVB_STV0288=m \
	CONFIG_DVB_TBSPRIV=m \
	CONFIG_DVB_STID135=m \
	CONFIG_DVB_MN88436=m \
	CONFIG_DVB_M88RS6060=m \
	CONFIG_DVB_STV0910=m \
	CONFIG_DVB_STV6111=m \
	CONFIG_DVB_CX24117=m \
	CONFIG_DVB_MXL58X=m \
	CONFIG_DVB_MXL5XX=m \
	CONFIG_DVB_GX1503=m \
	CONFIG_DVB_MTV23X=m \
	CONFIG_MEDIA_TUNER_SI2157=m \
	CONFIG_MEDIA_TUNER_RDA5816=m \
	CONFIG_MEDIA_TUNER_AV201X=m \
	CONFIG_MEDIA_TUNER_STV6120=m \
	CONFIG_MEDIA_TUNER_R848=m \
	CONFIG_MEDIA_TUNER_TDA18212=m \
	CONFIG_MEDIA_TUNER_TDA8290=m \
	CONFIG_MEDIA_TUNER_TDA18271=m \
	CONFIG_DVB_USB=m \
	CONFIG_DVB_USB_TBSQBOX=m \
	CONFIG_DVB_USB_TBSQBOX2=m \
	CONFIG_DVB_USB_TBSQBOX22=m \
	CONFIG_DVB_USB_TBS5520=m \
	CONFIG_DVB_USB_TBS5230=m \
	CONFIG_DVB_USB_TBS5220=m  \
	CONFIG_DVB_USB_TBS5301=m \
	CONFIG_DVB_USB_TBS5520se=m \
	CONFIG_DVB_USB_TBS5580=m \
	CONFIG_DVB_USB_TBS5590=m \
	CONFIG_DVB_USB_TBS5880=m \
	CONFIG_DVB_USB_TBS5881=m \
	CONFIG_DVB_USB_TBS5922SE=m \
	CONFIG_DVB_USB_TBS5925=m \
	CONFIG_DVB_USB_TBS5927=m \
	CONFIG_DVB_USB_TBS5930=m \
	CONFIG_DVB_USB_TBS5931=m \
	CONFIG_DVB_USB_TBS5530=m \
	CONFIG_DVB_MN88443X=m \
	CONFIG_MEDIA_TUNER_R820T=m \
	CONFIG_DVB_USB_V2=m \
	CONFIG_DVB_USB_RTL28XXU=m \
	CONFIG_DVB_RTL2832=m \
	CONFIG_DVB_RTL2832_SDR=m \
	CONFIG_DVB_MN88473=m \
	CONFIG_DVB_CXD2820=m \
	CONFIG_DVB_CXD2820R=m \
	CONFIG_DVB_NET=y


EXTRA_CFLAGS += --include=$(NEUMO_DIR)/include/kernel_compat.h -DCONFIG_DVB_MMAP
EXTRA_CFLAGS += -I$(NEUMO_DIR)/include \
                -I$(NEUMO_DIR)/include/linux \
                -I$(NEUMO_DIR)/dvb-core \
                -I$(NEUMO_DIR)/frontends \
                -I$(NEUMO_DIR)/frontends/stid135 \
                -I$(NEUMO_DIR)/tuners

EXTRA_CFLAGS += -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0
# --- Targets ---
.PHONY: all install clean dep

# Build modules using Kbuild M= syntax
all:
	$(MAKE) $(NEUMO_DIR)/include/linux/dvb/neumo-frontend.h $(NEUMO_DIR)/include/linux/dvb/neumo-dmx.h
	$(MAKE) $(NEUMO_DIR)/include/linux/dvb/neumo-frontend.h $(NEUMO_DIR)/dvb-core/neumo-version.c
	$(MAKE) -C $(KDIR) M=$(ROOT_DIR) $(MODDEFS) NOSTDINC_FLAGS="$(EXTRA_CFLAGS)" modules

$(NEUMO_DIR)/include/linux/dvb/neumo-frontend.h: $(NEUMO_DIR)/templates/common-frontend.h
	sed -s -e 's/common_//g'  -e 's/neumo_//g'  -e 's/COMMON_//g' \
	-e 's/NEUMO_//g' < $< > $@

$(NEUMO_DIR)/include/linux/dvb/neumo-dmx.h: $(NEUMO_DIR)/templates/common-dmx.h
	sed -s -e 's/common_//g'  -e 's/neumo_//g'  -e 's/COMMON_//g' \
	-e 's/NEUMO_//g' < $< > $@

$(NEUMO_DIR)/dvb-core/neumo-version.c::
	$(NEUMO_DIR)/update_version.sh

dep:
	$(MAKE) -C $(KDIR) M=$(ROOT_DIR) dep

install: all uninstall
	sudo $(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_PATH=$(MDIR) modules_install
	@echo "Updating module dependencies..."
	@if [ -n "$(MDIR)" ]; then \
		sudo depmod -b "$(MDIR)" -a $(KVER); \
	else \
		sudo depmod -a $(KVER); \
	fi

uninstall:
	sudo rm -fr  /lib/modules/$(KVER)/updates/extra/media # remove old neumo
	sudo rm -fr  /lib/modules/$(KVER)/updates/neumo
clean:
	@echo "Cleaning up..."
	@find . -type f -name "*.o" -o -name "*.ko" -o -name "*.mod" -o \
	              -name "*.mod.c" -o -name ".*.cmd" -o -name ".*.o.d" -o \
	              -name "*.order" -o -name "*.dwo" -o -name "modules.order" -o \
	              -name "Module.symvers" | xargs rm -f
	@rm -f $(NEUMO_DIR)/neumo/include/linux/dvb/neumo-frontend.h
	@rm -f $(NEUMO_DIR)/neumo/include/linux/dvb/neumo-dmx.h
	@rm -f $(NEUMO_DIR)/dvb-core/neumo-version.c
	@rm -rf .tmp_versions
