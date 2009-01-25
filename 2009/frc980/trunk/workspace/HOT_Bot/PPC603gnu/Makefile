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
PRJ_ROOT_DIR := C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot
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
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp
LIBPATH = 
LIBS = 

IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = $(WS_ROOT_DIR)/WPILib/PPC603gnu_DEBUG/WPILib.a 



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
PROJECT_TARGETS = HOT_Bot/$(MODE_DIR)/HOT_Bot.out


#Rules

# HOT_Bot
ifeq ($(DEBUG_MODE),1)
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
HOT_Bot/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
HOT_Bot/$(MODE_DIR)/% : IDE_INCLUDES = -I$(WIND_BASE)/target/h/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
HOT_Bot/$(MODE_DIR)/% : IDE_LIBRARIES = $(WS_ROOT_DIR)/WPILib/PPC603gnu_DEBUG/WPILib.a 
HOT_Bot/$(MODE_DIR)/% : PROJECT_TYPE = DKM
HOT_Bot/$(MODE_DIR)/% : DEFINES = 
HOT_Bot/$(MODE_DIR)/% : EXPAND_DBG = 0
HOT_Bot/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
HOT_Bot/$(MODE_DIR)/% : CPU = PPC603
HOT_Bot/$(MODE_DIR)/% : TOOL_FAMILY = gnu
HOT_Bot/$(MODE_DIR)/% : TOOL = gnu
HOT_Bot/$(MODE_DIR)/% : TOOL_PATH = 
HOT_Bot/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp
HOT_Bot/$(MODE_DIR)/% : LIBPATH = 
HOT_Bot/$(MODE_DIR)/% : LIBS = 
HOT_Bot/$(MODE_DIR)/% : OBJ_DIR := HOT_Bot/$(MODE_DIR)

HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/DashboardDataFormat.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot/DashboardDataFormat.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/DefaultRobot.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot/DefaultRobot.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/GamePad.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot/GamePad.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/HOT_Bot.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot/HOT_Bot.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/HOT_PID.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot/HOT_PID.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/TrackAPI.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/HOT_Bot/TrackAPI.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_HOT_Bot = HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/DashboardDataFormat.o \
	 HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/DefaultRobot.o \
	 HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/GamePad.o \
	 HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/HOT_Bot.o \
	 HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/HOT_PID.o \
	 HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/TrackAPI.o

HOT_Bot/$(MODE_DIR)/HOT_Bot.out : $(OBJECTS_HOT_Bot)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_HOT_Bot) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_HOT_Bot) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

HOT_Bot/$(MODE_DIR)/HOT_Bot_compile_file : $(FILE) ;

_clean :: HOT_Bot/$(MODE_DIR)/HOT_Bot_clean

HOT_Bot/$(MODE_DIR)/HOT_Bot_clean : 
	$(TRACE_FLAG)if [ -d "HOT_Bot" ]; then cd "HOT_Bot"; rm -rf $(MODE_DIR); fi

DEP_FILES := HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/DashboardDataFormat.d HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/DefaultRobot.d HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/GamePad.d \
	 HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/HOT_Bot.d HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/HOT_PID.d HOT_Bot/$(MODE_DIR)/Objects/HOT_Bot/TrackAPI.d
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
