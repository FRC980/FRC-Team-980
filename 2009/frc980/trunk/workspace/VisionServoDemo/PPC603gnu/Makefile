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
PRJ_ROOT_DIR := C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/VisionServoDemo
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
CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
LIBPATH = 
LIBS = 

IDE_INCLUDES = -IC:/Users/shadow/projects/FIRST/2009/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 

IDE_LIBRARIES = C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/WPILib/PPC603gnu_DEBUG/WPILib.a 



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
PROJECT_TARGETS = VisionServoDemo/$(MODE_DIR)/VisionServoDemo.out \
	 VisionServoDemo_partialImage/$(MODE_DIR)/VisionServoDemo_partialImage.o


#Rules

# VisionServoDemo
ifeq ($(DEBUG_MODE),1)
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
VisionServoDemo/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
VisionServoDemo/$(MODE_DIR)/% : IDE_INCLUDES = -IC:/Users/shadow/projects/FIRST/2009/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
VisionServoDemo/$(MODE_DIR)/% : IDE_LIBRARIES = C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/WPILib/PPC603gnu_DEBUG/WPILib.a 
VisionServoDemo/$(MODE_DIR)/% : PROJECT_TYPE = DKM
VisionServoDemo/$(MODE_DIR)/% : DEFINES = 
VisionServoDemo/$(MODE_DIR)/% : EXPAND_DBG = 0
VisionServoDemo/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
VisionServoDemo/$(MODE_DIR)/% : CPU = PPC603
VisionServoDemo/$(MODE_DIR)/% : TOOL_FAMILY = gnu
VisionServoDemo/$(MODE_DIR)/% : TOOL = gnu
VisionServoDemo/$(MODE_DIR)/% : TOOL_PATH = 
VisionServoDemo/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
VisionServoDemo/$(MODE_DIR)/% : LIBPATH = 
VisionServoDemo/$(MODE_DIR)/% : LIBS = 
VisionServoDemo/$(MODE_DIR)/% : OBJ_DIR := VisionServoDemo/$(MODE_DIR)

OBJECTS_VisionServoDemo = VisionServoDemo_partialImage/$(MODE_DIR)/VisionServoDemo_partialImage.o

VisionServoDemo/$(MODE_DIR)/VisionServoDemo.out : $(OBJECTS_VisionServoDemo)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@";rm -f "$@";nmppc $(OBJECTS_VisionServoDemo) $(IDE_LIBRARIES) | tclsh $(WIND_BASE)/host/resource/hutils/tcl/munch.tcl -c ppc > $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc $(DEBUGFLAGS_Linker) $(CC_ARCH_SPEC) -fdollars-in-identifiers -Wall $(ADDED_CFLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES)  -o $(OBJ_DIR)/ctdt.o -c $(OBJ_DIR)/ctdt.c; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X -T $(WIND_BASE)/target/h/tool/gnu/ldscripts/link.OUT -o "$@" $(OBJ_DIR)/ctdt.o $(OBJECTS_VisionServoDemo) $(IDE_LIBRARIES) $(LIBPATH) $(LIBS)  $(ADDED_LIBPATH) $(ADDED_LIBS) && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

VisionServoDemo/$(MODE_DIR)/VisionServoDemo_compile_file : $(FILE) ;

_clean :: VisionServoDemo/$(MODE_DIR)/VisionServoDemo_clean

VisionServoDemo/$(MODE_DIR)/VisionServoDemo_clean : 
	$(TRACE_FLAG)if [ -d "VisionServoDemo" ]; then cd "VisionServoDemo"; rm -rf $(MODE_DIR); fi


# VisionServoDemo_partialImage
ifeq ($(DEBUG_MODE),1)
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -g
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -g
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -g
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -g
else
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C-Compiler = -O2 -fstrength-reduce -fno-builtin
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_C++-Compiler = -O2 -fstrength-reduce -fno-builtin
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Linker = -O2 -fstrength-reduce -fno-builtin
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Partial-Image-Linker = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Librarian = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEBUGFLAGS_Assembler = -O2 -fstrength-reduce -fno-builtin
endif
VisionServoDemo_partialImage/$(MODE_DIR)/% : IDE_INCLUDES = -IC:/Users/shadow/projects/FIRST/2009/WPILib -I$(WIND_BASE)/target/h -I$(WIND_BASE)/target/h/wrn/coreip 
VisionServoDemo_partialImage/$(MODE_DIR)/% : IDE_LIBRARIES = C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/WPILib/PPC603gnu_DEBUG/WPILib.a 
VisionServoDemo_partialImage/$(MODE_DIR)/% : PROJECT_TYPE = DKM
VisionServoDemo_partialImage/$(MODE_DIR)/% : DEFINES = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : EXPAND_DBG = 0
VisionServoDemo_partialImage/$(MODE_DIR)/% : VX_CPU_FAMILY = ppc
VisionServoDemo_partialImage/$(MODE_DIR)/% : CPU = PPC603
VisionServoDemo_partialImage/$(MODE_DIR)/% : TOOL_FAMILY = gnu
VisionServoDemo_partialImage/$(MODE_DIR)/% : TOOL = gnu
VisionServoDemo_partialImage/$(MODE_DIR)/% : TOOL_PATH = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : CC_ARCH_SPEC = -mcpu=603 -mstrict-align -mno-implicit-fp -mlongcall
VisionServoDemo_partialImage/$(MODE_DIR)/% : LIBPATH = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : LIBS = 
VisionServoDemo_partialImage/$(MODE_DIR)/% : OBJ_DIR := VisionServoDemo_partialImage/$(MODE_DIR)

VisionServoDemo_partialImage/$(MODE_DIR)/Objects/VisionServoDemo/VisionServoDemo.o : C:/Users/shadow/projects/FIRST/2009/frc980/trunk/workspace/VisionServoDemo/VisionServoDemo.cpp
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc $(DEBUGFLAGS_C++-Compiler) $(CC_ARCH_SPEC) -ansi -Wall  -MD -MP -mlongcall $(ADDED_C++FLAGS) $(IDE_INCLUDES) $(ADDED_INCLUDES) -DCPU=$(CPU) -DTOOL_FAMILY=$(TOOL_FAMILY) -DTOOL=$(TOOL) -D_WRS_KERNEL   $(DEFINES) -o "$@" -c "$<"


OBJECTS_VisionServoDemo_partialImage = VisionServoDemo_partialImage/$(MODE_DIR)/Objects/VisionServoDemo/VisionServoDemo.o

VisionServoDemo_partialImage/$(MODE_DIR)/VisionServoDemo_partialImage.o : $(OBJECTS_VisionServoDemo_partialImage)
	$(TRACE_FLAG)if [ ! -d "`dirname "$@"`" ]; then mkdir -p "`dirname "$@"`"; fi;echo "building $@"; $(TOOL_PATH)ccppc -r -nostdlib -Wl,-X  -o "$@" $(OBJECTS_VisionServoDemo_partialImage) $(LIBPATH) $(LIBS) $(ADDED_LIBPATH) $(ADDED_LIBS)  && if [ "$(EXPAND_DBG)" = "1" ]; then plink "$@";fi

VisionServoDemo_partialImage/$(MODE_DIR)/VisionServoDemo_partialImage_compile_file : $(FILE) ;

_clean :: VisionServoDemo_partialImage/$(MODE_DIR)/VisionServoDemo_partialImage_clean

VisionServoDemo_partialImage/$(MODE_DIR)/VisionServoDemo_partialImage_clean : 
	$(TRACE_FLAG)if [ -d "VisionServoDemo_partialImage" ]; then cd "VisionServoDemo_partialImage"; rm -rf $(MODE_DIR); fi

DEP_FILES := VisionServoDemo_partialImage/$(MODE_DIR)/Objects/VisionServoDemo/VisionServoDemo.d
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
