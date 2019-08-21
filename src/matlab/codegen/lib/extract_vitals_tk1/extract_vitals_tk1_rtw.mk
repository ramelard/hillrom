###########################################################################
## Makefile generated for MATLAB file/project 'extract_vitals_tk1'. 
## 
## Makefile     : extract_vitals_tk1_rtw.mk
## Generated on : Wed Mar 27 12:58:56 2019
## MATLAB Coder version: 4.0 (R2018a)
## 
## Build Info:
## 
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/extract_vitals_tk1.lib
## Product type : static-library
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPUTER                Computer type. See the MATLAB "computer" command.

PRODUCT_NAME              = extract_vitals_tk1
MAKEFILE                  = extract_vitals_tk1_rtw.mk
COMPUTER                  = PCWIN64
MATLAB_ROOT               = C:/PROGRA~1/MATLAB/R2018a
MATLAB_BIN                = C:/PROGRA~1/MATLAB/R2018a/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/win64
MASTER_ANCHOR_DIR         = 
START_DIR                 = C:/Users/ramelard/Documents/hillrom-master/hillrom-master/src/matlab
ARCH                      = win64
RELATIVE_PATH_TO_ANCHOR   = .
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Linaro Toolchain v4.8 | gmake (64-bit Windows)
# Supported Version(s):    
# ToolchainInfo Version:   R2018a
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# TARGET_LOAD_CMD_ARGS
# TARGET_PKG_INSTALLDIR
# LINARO_TOOLCHAIN_4_8

#-----------
# MACROS
#-----------

SHELL                 = %SystemRoot%/system32/cmd.exe
CCOUTPUTFLAG          = --output_file=
LDOUTPUTFLAG          = --output_file=

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lm

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: Linaro Toolchain4.8 Assembler
AS_PATH = $(LINARO_TOOLCHAIN_4_8)
AS = $(AS_PATH)/arm-linux-gnueabihf-as

# C Compiler: Linaro Toolchain4.8 C Compiler
CC_PATH = $(LINARO_TOOLCHAIN_4_8)
CC = $(CC_PATH)/arm-linux-gnueabihf-gcc

# Linker: Linaro Toolchain4.8 Linker
LD_PATH = $(LINARO_TOOLCHAIN_4_8)
LD = $(LD_PATH)/arm-linux-gnueabihf-gcc

# C++ Compiler: Linaro Toolchain4.8 C++ Compiler
CPP_PATH = $(LINARO_TOOLCHAIN_4_8)
CPP = $(CPP_PATH)/arm-linux-gnueabihf-g++

# C++ Linker: Linaro Toolchain4.8 C++ Linker
CPP_LD_PATH = $(LINARO_TOOLCHAIN_4_8)
CPP_LD = $(CPP_LD_PATH)/arm-linux-gnueabihf-g++

# Archiver: Linaro Toolchain4.8 Archiver
AR_PATH = $(LINARO_TOOLCHAIN_4_8)
AR = $(AR_PATH)/arm-linux-gnueabihf-ar

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = $(MEX_PATH)/mex

# Download: Download
DOWNLOAD_PATH = $(TARGET_PKG_INSTALLDIR)
DOWNLOAD = $(DOWNLOAD_PATH)/ssh_download.bat

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: GMAKE Utility
MAKE_PATH = %MATLAB%\bin\win64
MAKE = $(MAKE_PATH)/gmake


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  = @del /F
ECHO                = @echo
MV                  = @move
RUN                 =

#----------------------------------------
# "Faster Builds" Build Configuration
#----------------------------------------

ARFLAGS              = -ruvs
ASFLAGS              = -c \
                       $(ASFLAGS_ADDITIONAL) \
                       $(INCLUDES)
CFLAGS               = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -O0
CPPFLAGS             = -c \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -O2
CPP_LDFLAGS          = -lm -lrt -lpthread -ldl
CPP_SHAREDLIB_LDFLAGS  = -shared  \
                         -lm -lrt -lpthread -ldl
DOWNLOAD_FLAGS       = $(TARGET_LOAD_CMD_ARGS) $(PRODUCT)
EXECUTE_FLAGS        =
LDFLAGS              = -lm -lrt -lpthread -ldl
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    = -shared  \
                       -lm -lrt -lpthread -ldl

#--------------------
# File extensions
#--------------------

ASM_EXT             = .s.dep
ASMOBJ_EXT          = .s.o
ASM_EXT             = .s
C_EXT               = .c.dep
H_EXT               = .h
COBJ_EXT            = .c.o
C_EXT               = .c
EXE_EXT             = .elf
SHAREDLIB_EXT       = .so
CPP_EXT             = .cpp.dep
HPP_EXT             = .hpp
CPPOBJ_EXT          = .pp.o
CPP_EXT             = .cpp
EXE_EXT             =
SHAREDLIB_EXT       = .so
STATICLIB_EXT       = .lib
MEX_EXT             = .mexw64
MAKE_EXT            = .mk


###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/extract_vitals_tk1.lib
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR)/codegen/lib/extract_vitals_tk1 -I$(START_DIR) -IC:/ProgramData/MATLAB/SupportPackages/R2018a/toolbox/target/supportpackages/arm_cortex_a/include -IC:/ProgramData/MATLAB/SupportPackages/R2018a/toolbox/target/supportpackages/arm_cortex_a/blocks/sfcn/include -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DARM_PROJECT -DSTACK_SIZE=200000 -DMODEL=extract_vitals_tk1 -DHAVESTDIO -DUSE_RTMODEL
DEFINES_SKIPFORSIL = -DARM_PROJECT -DSTACK_SIZE=200000
DEFINES_STANDARD = -DMODEL=extract_vitals_tk1 -DHAVESTDIO -DUSE_RTMODEL

DEFINES = $(DEFINES_) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/codegen/lib/extract_vitals_tk1/extract_vitals_tk1_rtwutil.c $(START_DIR)/codegen/lib/extract_vitals_tk1/extract_vitals_tk1_initialize.c $(START_DIR)/codegen/lib/extract_vitals_tk1/extract_vitals_tk1_terminate.c $(START_DIR)/codegen/lib/extract_vitals_tk1/extract_vitals_tk1.c $(START_DIR)/codegen/lib/extract_vitals_tk1/floor.c $(START_DIR)/codegen/lib/extract_vitals_tk1/rdivide.c $(START_DIR)/codegen/lib/extract_vitals_tk1/permute.c $(START_DIR)/codegen/lib/extract_vitals_tk1/interp1.c $(START_DIR)/codegen/lib/extract_vitals_tk1/log.c $(START_DIR)/codegen/lib/extract_vitals_tk1/fft.c $(START_DIR)/codegen/lib/extract_vitals_tk1/fft1.c $(START_DIR)/codegen/lib/extract_vitals_tk1/get_spectral_entropy.c $(START_DIR)/codegen/lib/extract_vitals_tk1/sum.c $(START_DIR)/codegen/lib/extract_vitals_tk1/combineVectorElements.c $(START_DIR)/codegen/lib/extract_vitals_tk1/log21.c $(START_DIR)/codegen/lib/extract_vitals_tk1/extract_vitals_tk1_emxutil.c $(START_DIR)/codegen/lib/extract_vitals_tk1/extract_vitals_tk1_emxAPI.c $(START_DIR)/codegen/lib/extract_vitals_tk1/rt_nonfinite.c $(START_DIR)/codegen/lib/extract_vitals_tk1/rtGetNaN.c $(START_DIR)/codegen/lib/extract_vitals_tk1/rtGetInf.c

ALL_SRCS = $(SRCS)

###########################################################################
## OBJECTS
###########################################################################

OBJS = extract_vitals_tk1_rtwutil.c.o extract_vitals_tk1_initialize.c.o extract_vitals_tk1_terminate.c.o extract_vitals_tk1.c.o floor.c.o rdivide.c.o permute.c.o interp1.c.o log.c.o fft.c.o fft1.c.o get_spectral_entropy.c.o sum.c.o combineVectorElements.c.o log21.c.o extract_vitals_tk1_emxutil.c.o extract_vitals_tk1_emxAPI.c.o rt_nonfinite.c.o rtGetNaN.c.o rtGetInf.c.o

ALL_OBJS = $(OBJS)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_SKIPFORSIL = -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a9
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_SKIPFORSIL) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_SKIPFORSIL = -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a9
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_SKIPFORSIL) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_SKIPFORSIL = -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a9

CPP_LDFLAGS += $(CPP_LDFLAGS_SKIPFORSIL)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL = -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a9

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL)

#-----------
# Linker
#-----------

LDFLAGS_SKIPFORSIL = -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a9

LDFLAGS += $(LDFLAGS_SKIPFORSIL)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_SKIPFORSIL = -march=armv7-a -mfloat-abi=hard -mfpu=neon -mtune=cortex-a9

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_SKIPFORSIL)

###########################################################################
## INLINED COMMANDS
###########################################################################


ALL_DEPS:=$(patsubst %.o,%.dep,$(ALL_OBJS))
all:




-include codertarget_assembly_flags.mk
-include ../codertarget_assembly_flags.mk
-include $(ALL_DEPS)


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : build


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.c.o : %.c
	$(CC) $(CFLAGS) -o $@ $<


%.s.o : %.s
	$(AS) $(ASFLAGS) -o $@ $<


%.pp.o : %.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.c.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.s.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o $@ $<


%.pp.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.c.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.s.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o $@ $<


%.pp.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.c.o : $(START_DIR)/codegen/lib/extract_vitals_tk1/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.s.o : $(START_DIR)/codegen/lib/extract_vitals_tk1/%.s
	$(AS) $(ASFLAGS) -o $@ $<


%.pp.o : $(START_DIR)/codegen/lib/extract_vitals_tk1/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


%.c.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o $@ $<


%.s.o : $(MATLAB_ROOT)/rtw/c/src/%.s
	$(AS) $(ASFLAGS) -o $@ $<


%.pp.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o $@ $<


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : $(MAKEFILE) rtw_proj.tmw


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "### PRODUCT = $(PRODUCT)"
	@echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "### BUILD_TYPE = $(BUILD_TYPE)"
	@echo "### INCLUDES = $(INCLUDES)"
	@echo "### DEFINES = $(DEFINES)"
	@echo "### ALL_SRCS = $(ALL_SRCS)"
	@echo "### ALL_OBJS = $(ALL_OBJS)"
	@echo "### LIBS = $(LIBS)"
	@echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "### ASFLAGS = $(ASFLAGS)"
	@echo "### CFLAGS = $(CFLAGS)"
	@echo "### LDFLAGS = $(LDFLAGS)"
	@echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "### CPPFLAGS = $(CPPFLAGS)"
	@echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	@echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	@echo "### ARFLAGS = $(ARFLAGS)"
	@echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	@echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	@echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files..."
	$(RM) $(subst /,\,$(PRODUCT))
	$(RM) $(subst /,\,$(ALL_OBJS))
	$(RM) *.c.dep
	$(RM) *.cpp.dep
	$(RM) *.s.dep
	$(ECHO) "### Deleted all derived files."


