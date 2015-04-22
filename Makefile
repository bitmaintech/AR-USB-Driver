#
# Copyright (C) 2008 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/kernel.mk

PKG_NAME:=bitmain-asic
PKG_RELEASE:=1

include $(INCLUDE_DIR)/package.mk

define KernelPackage/bitmain-asic
  SUBMENU:=Other modules
  TITLE:=bitmain asic device
  DEPENDS:= +kmod-usb-core
  #FILES:=$(PKG_BUILD_DIR)/bitmain-asic.ko $(PKG_BUILD_DIR)/usb-bitmain.ko $(PKG_BUILD_DIR)/sha2.ko
  FILES:=$(PKG_BUILD_DIR)/bitmain-asic.ko $(PKG_BUILD_DIR)/usb-bitmain.ko $(PKG_BUILD_DIR)/pic-update.ko 
  KCONFIG:=
  AUTOLOAD:=$(call AutoLoad,30,bitmain-asic pic-update,1)

endef

define KernelPackage/bitmain-asic/description
 Kernel module for register a bitmain asic device.
endef

EXTRA_KCONFIG:= \
	CONFIG_BITMAIN_ASIC=m

EXTRA_CFLAGS:= \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=m,%,$(filter %=m,$(EXTRA_KCONFIG)))) \
	$(patsubst CONFIG_%, -DCONFIG_%=1, $(patsubst %=y,%,$(filter %=y,$(EXTRA_KCONFIG)))) \

MAKE_OPTS:= \
	ARCH="$(LINUX_KARCH)" \
	CROSS_COMPILE="$(TARGET_CROSS)" \
	SUBDIRS="$(PKG_BUILD_DIR)" \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(EXTRA_KCONFIG)

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) ./src/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C "$(LINUX_DIR)" \
		$(MAKE_OPTS) \
		modules
endef

$(eval $(call KernelPackage,bitmain-asic))
