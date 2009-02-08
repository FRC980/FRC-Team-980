# Wind River Workbench generated Makefile.
# Do not edit!!!
#
# The file ".wrmakefile" is the template used by the Wind River Workbench to
# generate the makefiles of this project. Add user-specific build targets and
# make rules only(!) in this project's ".wrmakefile" file. These will then be
# automatically dumped into the makefiles.

WIND_HOME := $(subst \,/,$(WIND_HOME))
WIND_BASE := $(subst \,/,$(WIND_BASE))
WIND_USR := $(subst \,/,$(WIND_USR))

all : pre_build main_all post_build

_clean ::
	@echo "make: removing targets and objects of `pwd`"

TRACE=0
TRACEON=$(TRACE:0=@)
TRACE_FLAG=$(TRACEON:1=)

MAKEFILE := Makefile

FLEXIBLE_BUILD := 1

BUILD_SPEC = PPC603gnu
DEBUG_MODE = 1
ifeq ($(DEBUG_MODE),1)
MODE_DIR := Debug
else
MODE_DIR := NonDebug
endif
OBJ_DIR := .
PRJ_ROOT_DIR := C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/FRC980
WS_ROOT_DIR := C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace



#Global Build Macros
PROJECT_TYPE = DKM
DEFINES = 
EXPAND_DBG = 0


#BuildSpec specific Build Macros
VX_CPU_FAMILY = ppc
CPU = PPC603
TOOL_FAMILY = gnu
TOOL = gnu
TOOL_PATH = 
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp  -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip -I. 

IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 



#BuildTool flags
ifeq ($(DEBUG_MODE),1)
DEBUGFLAGS_C-Compiler = -g
DEBUGFLAGS_C++-Compiler = -g
DEBUGFLAGS_Linker = -g
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -g
else
DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
DEBUGFLAGS_Partial-Image-Linker = 
DEBUGFLAGS_Librarian = 
DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif


#Project Targets
PROJECT_TARGETS = FRC980/$(MODE_DIR)/FRC980.out \
	 FRC980_partialImage/$(MODE_DIR)/FRC980_partialImage.o


#Rules

# FRC980
ifeq ($(DEBUG_MODE),1)
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FRC980/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
FRC980/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip -I. 
FRC980/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
FRC980/$(MODE_DIR)/% : PROJECT_TYPE = DKM
FRC980/$(MODE_DIR)/% : DEFINES = 
FRC980/$(MODE_DIR)/% : EXPAND_DBG = 0
FRC980/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
FRC980/$(MODE_DIR)/% : CPU = PPC603
FRC980/$(MODE_DIR)/% : TOOL_FAMILY = gnu
FRC980/$(MODE_DIR)/% : TOOL = gnu
FRC980/$(MODE_DIR)/% : TOOL_PATH = 
FRC980/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp  -mlongcall
FRC980/$(MODE_DIR)/% : LIBPATH = 
FRC980/$(MODE_DIR)/% : LIBS = 
FRC980/$(MODE_DIR)/% : OBJ_DIR := FRC980/$(MODE_DIR)

OBJECTS_FRC980 = FRC980_partialImage/$(MODE_DIR)/FRC980_partialImage.o

FRC980/$(MODE_DIR)/FRC980.out : $(OBJECTS_FRC980)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_FRC980) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_FRC980) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

FRC980/$(MODE_DIR)/FRC980_compile_file : $(FILE) ;

_clean :: FRC980/$(MODE_DIR)/FRC980_clean

FRC980/$(MODE_DIR)/FRC980_clean : 
	$(TRACE_FLAG)if [ -d "FRC980" ]; then cd "FRC980"; rm -rf $(MODE_DIR); fi


# FRC980_partialImage
ifeq ($(DEBUG_MODE),1)
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
FRC980_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
FRC980_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h/wrn/coreip -I. 
FRC980_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = $(WIND_BASE)/target/lib/WPILib.a 
FRC980_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
FRC980_partialImage/$(MODE_DIR)/% : DEFINES = 
FRC980_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
FRC980_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
FRC980_partialImage/$(MODE_DIR)/% : CPU = PPC603
FRC980_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
FRC980_partialImage/$(MODE_DIR)/% : TOOL = gnu
FRC980_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
FRC980_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp  -mlongcall
FRC980_partialImage/$(MODE_DIR)/% : LIBPATH = 
FRC980_partialImage/$(MODE_DIR)/% : LIBS = 
FRC980_partialImage/$(MODE_DIR)/% : OBJ_DIR := FRC980_partialImage/$(MODE_DIR)

FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/DashboardData.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/FRC980/DashboardData.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/DashboardDataExample.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/FRC980/DashboardDataExample.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/Robot980.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/FRC980/Robot980.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/SmartDrive.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/FRC980/SmartDrive.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/main.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/FRC980/main.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_FRC980_partialImage = FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/DashboardData.o \
	 FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/DashboardDataExample.o \
	 FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/Robot980.o \
	 FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/SmartDrive.o \
	 FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/main.o

FRC980_partialImage/$(MODE_DIR)/FRC980_partialImage.o : $(OBJECTS_FRC980_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_FRC980_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

FRC980_partialImage/$(MODE_DIR)/FRC980_partialImage_compile_file : $(FILE) ;

_clean :: FRC980_partialImage/$(MODE_DIR)/FRC980_partialImage_clean

FRC980_partialImage/$(MODE_DIR)/FRC980_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "FRC980_partialImage" ]; then cd "FRC980_partialImage"; rm -rf $(MODE_DIR); fi

DEP_FILES := FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/DashboardData.d FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/DashboardDataExample.d FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/Robot980.d \
	 FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/SmartDrive.d FRC980_partialImage/$(MODE_DIR)/Objects/FRC980/main.d
-include $(DEP_FILES)



-include $(PRJ_ROOT_DIR)/*.makefile

-include *.makefile

main_all : external_build $(PROJECT_TARGETS)
	@echo "make: built targets of `pwd`"

# entry point for extending the build
external_build ::
	@echo ""

# main entry point for pre processing prior to the build
pre_build :: $(PRE_BUILD_STEP) generate_sources
	@echo ""

# entry point for generating sources prior to the build
generate_sources ::
	@echo ""

# main entry point for post processing after the build
post_build :: $(POST_BUILD_STEP) deploy_output
	@echo ""

# entry point for deploying output after the build
deploy_output ::
	@echo ""

clean :: external_clean $(CLEAN_STEP) _clean

# entry point for extending the build clean
external_clean ::
	@echo ""
