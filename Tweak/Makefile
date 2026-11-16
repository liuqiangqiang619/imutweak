TARGET := iphone:clang:16.5:15.0
ARCHS := arm64

include $(THEOS)/makefiles/common.mk

TWEAK_NAME = IMUOnly
IMUOnly_FILES = IMUOnly.m
IMUOnly_CFLAGS = -fobjc-arc
IMUOnly_FRAMEWORKS = Foundation UIKit CoreMotion QuartzCore
IMUOnly_LIBRARIES =

include $(THEOS_MAKE_PATH)/tweak.mk
