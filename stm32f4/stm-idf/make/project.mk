# Main Project Makefile 
# This Makefile is included directly from the user project Makefile in order to call the component.mk
# makefiles of all components (in a seperate make process) to build all libraries, then links them 
# together into the final file. 

.PHONY: build flash clean monitor help

help:
	@echo "Welcom to STM-IDF. There are some make targets:"
	@echo ""
	@echo "make build - Build project"
	@echo "make clean - Remove all build output"
	@echo "make flash - Flash to stm32 targets via ST-LinkV2"
	@echo "make monitor - View log output"
	@echo ""
	@echo "Visit https://github.com/thanhphong98/stm-idf to see more details about STM-IDF or contribute"
	@echo "Visit https://github.com/thanhphong98/stm32-template to get project template"

# Default path of the project. Assume the Makefile is exist in current project directory.
ifndef PROJECT_PATH
PROJECT_PATH := $(abspath $(dir $(firstword $(MAKEFILE_LIST))))
export PROJECT_PATH
endif

# The directory where we put all binaries. The project Makefile can configure it if needed.
ifndef BUILD_DIR
	BUILD_DIR := $(PROJECT_PATH)/build
export BUILD_DIR
endif

# If no configure STM_IDF_PATH to variable environment, use stm-idf in current project. Assume
# current project contain stm-idf
ifndef STM_IDF_PATH
STM_IDF_PATH := $(PROJECT_PATH)/stm-idf
export STM_IDF_PATH
endif

# Component directory. The project Makefile can override these directory, or add extra component
# directory via EXTRA_COMPONENT_DIRS
ifndef COMPONENT_DIRS
EXTRA_COMPONENT_DIRS ?=
COMPONENT_DIRS := $(PROJECT_PATH)/components $(EXTRA_COMPONENT_DIRS) $(STM_IDF_PATH)/components $(PROJECT_PATH)/main
endif

# Make sure that every directory in the list is absulute path without trailing slash.
COMPONENT_DIRS := $(foreach cd,$(COMPONENT_DIRS),$(abspath $(cd)))
export COMPONENT_DIRS

# This is neccessary to split COMPONET_DIRS into SINGLE_COMPONET_DIRS and MULTI_COMPONENT_DIRS.
# SINGLE_COMPONENT_DIRS contain a component.mk file and MULTI_COMPONENT_DIRS contain folder which
# contrain component.mk file. For example /blablabla/components/user_components/component.mk
SINGLE_COMPONENT_DIRS := $(abspath $(dir $(dir $(foreach cd, $(COMPONENT_DIRS), $(wildcard $(cd)/component.mk)))))
export SINGLE_COMPONENT_DIRS
MULTI_COMPONENT_DIRS := $(filter-out $(SINGLE_COMPONENT_DIRS),$(COMPONENT_DIRS))

# Find all component names (which folder contain component.mk file).
# We need to do this for MULTI_COMPONENT_DIRS only, since SINGLE_COMPONENT_DIRS
# are already known to contain component.mk.
ifndef COMPONENTS
COMPONENTS := $(dir $(foreach cd,$(MULTI_COMPONENT_DIRS),$(wildcard $(cd)/*/component.mk))) $(SINGLE_COMPONENT_DIRS)
COMPONENTS := $(sort $(foreach comp,$(COMPONENTS),$(lastword $(subst /, ,$(comp)))))
endif
export COMPONENTS

# Resolve all of COMPONENTS into absolute paths in COMPONENT_PATHS.
# For each entry in COMPONENT_DIRS:
# - either this is directory with multiple components, in which case check that
#   a subdirectory with component name exists, and it contains a component.mk file.
# - or, this is a directory of a single component, in which case the name of this
#   directory has to match the component name
#
# If a component name exists in multiple COMPONENT_DIRS, we take the first match.
#
# NOTE: These paths must be generated WITHOUT a trailing / so we
# can use $(notdir x) to get the component name.
COMPONENT_PATHS := $(foreach comp,$(COMPONENTS),\
                        $(firstword $(foreach cd,$(COMPONENT_DIRS),\
                            $(if $(findstring $(cd),$(MULTI_COMPONENT_DIRS)),\
                                 $(abspath $(dir $(wildcard $(cd)/$(comp)/component.mk))),)\
                            $(if $(findstring $(cd),$(SINGLE_COMPONENT_DIRS)),\
                                 $(if $(filter $(comp),$(notdir $(cd))),$(cd),),)\
                   )))
export COMPONENT_PATHS

# Default include and source directory in components folder.
# 	- COMPONENT_INCLUDES: include directory regarless include folder (which set default).
#	- COMPONENT_SOURCES: source directory regarless self directory.
COMPONENT_INCLUDES += include
COMPONENT_SOURCES += 

# Get all variable in every components. This variable include COMPONENT_INCLUDES and COMPONENT_SOURCES.
include $(foreach comp, $(COMPONENT_PATHS), \
						$(addprefix $(comp)/, component.mk))

# Add component include prefix paths to componnent paths to get all absolute include paths. 
INCLUDE_PATHS += $(foreach comp, $(COMPONENT_PATHS), \
					$(foreach comp_inc, $(COMPONENT_INCLUDES), \
						$(addprefix -I$(comp)/, $(comp_inc))))

# Add component source prefix paths to component paths to get all absolute source paths.
SOURCE_PATHS += $(COMPONENT_PATHS)
SOURCE_PATHS += $(foreach comp, $(COMPONENT_PATHS), \
					$(foreach comp_src, $(COMPONENT_SOURCES), \
						$(addprefix $(comp)/, $(comp_src))))

# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
CXX = $(GCC_PATH)/$(PREFIX)g++
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
CXX = $(PREFIX)g++
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# debug build?
DEBUG = 1
OPT = -Og
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)
AS_DEFS = 
C_DEFS =  \
-DUSE_HAL_DRIVER \
-DSTM32F407xx

# ASM flags
ASFLAGS = $(MCU) $(AS_DEFS) $(INCLUDE_PATHS) $(OPT) -Wall -fdata-sections -ffunction-sections

# C flags
CFLAGS = $(MCU) $(C_DEFS) $(INCLUDE_PATHS) $(OPT) -Wall -fdata-sections -ffunction-sections 
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" 

# C++ flags
CXXFLAGS = $(MCU) $(C_DEFS) $(INCLUDE_PATHS) $(OPT) -Wall -fdata-sections -ffunction-sections
CXXFLAGS += -MMD -MP -MF"$(@:%.o=%.d)" 

# LD flags
LDSCRIPT = stm-idf/make/stm32f4xx_flash.ld
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nosys.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections
LDFLAGS += -fno-exceptions -fno-rtti 
# Get all source files include .c and .s
C_SOURCES += $(foreach comp_src, $(SOURCE_PATHS), $(wildcard $(comp_src)/*.c))
CPP_SOURCES += $(foreach comp_src, $(SOURCE_PATHS), $(wildcard $(comp_src)/*.cpp))
ASM_SOURCES += $(foreach comp_src, $(SOURCE_PATHS), $(wildcard $(comp_src)/*.s))

# Target name
TARGET = $(PROJECT_NAME)

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of c++ program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CPP_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# Build application
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)
  
#######################################
# flash
#######################################
flash:
	st-flash write build/$(PROJECT_NAME).bin 0x8000000
  
#######################################
# monitor 
#######################################
monitor:
	minicom -c on
  

