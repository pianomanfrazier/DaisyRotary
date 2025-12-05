# Project Name
TARGET = DaisyRotary

USE_DAISYSP_LGPL = 1

# Sources
CPP_SOURCES = src/main.cpp src/leslie.cpp src/preamp.cpp

C_INCLUDES += -Isrc

# Library Locations
LIBDAISY_DIR = ../DaisyExamples/libDaisy/
DAISYSP_DIR = ../DaisyExamples/DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
