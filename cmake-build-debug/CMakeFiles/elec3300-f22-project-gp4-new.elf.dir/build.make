# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.23

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj: ../Core/Src/lcdtp.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/lcdtp.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/lcdtp.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/lcdtp.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj: ../Core/Src/main.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/main.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/main.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/main.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj: ../Core/Src/stm32f1xx_hal_msp.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/stm32f1xx_hal_msp.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/stm32f1xx_hal_msp.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/stm32f1xx_hal_msp.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj: ../Core/Src/stm32f1xx_it.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/stm32f1xx_it.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/stm32f1xx_it.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/stm32f1xx_it.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj: ../Core/Src/syscalls.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/syscalls.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/syscalls.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/syscalls.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj: ../Core/Src/sysmem.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/sysmem.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/sysmem.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/sysmem.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj: ../Core/Src/system_stm32f1xx.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/system_stm32f1xx.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/system_stm32f1xx.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/system_stm32f1xx.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj: ../Core/Src/xpt2046.c
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj -MF CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj.d -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/xpt2046.c

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.i"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/xpt2046.c > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.s"
	/usr/local/bin/arm-none-eabi-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Src/xpt2046.c -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.obj: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/flags.make
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.obj: ../Core/Startup/startup_stm32f103vetx.s
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building ASM object CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.obj"
	/usr/local/bin/arm-none-eabi-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.obj -c /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Startup/startup_stm32f103vetx.s

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing ASM source to CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.i"
	/usr/local/bin/arm-none-eabi-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -E /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Startup/startup_stm32f103vetx.s > CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.i

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling ASM source to assembly CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.s"
	/usr/local/bin/arm-none-eabi-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -S /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/Core/Startup/startup_stm32f103vetx.s -o CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.s

# Object files for target elec3300-f22-project-gp4-new.elf
elec3300__f22__project__gp4__new_elf_OBJECTS = \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj" \
"CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.obj"

# External object files for target elec3300-f22-project-gp4-new.elf
elec3300__f22__project__gp4__new_elf_EXTERNAL_OBJECTS =

elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/lcdtp.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/main.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_hal_msp.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/stm32f1xx_it.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/syscalls.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/sysmem.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/system_stm32f1xx.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Src/xpt2046.c.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/Core/Startup/startup_stm32f103vetx.s.obj
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/build.make
elec3300-f22-project-gp4-new.elf: CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking C executable elec3300-f22-project-gp4-new.elf"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Building /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/elec3300-f22-project-gp4-new.hex"
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Building /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/elec3300-f22-project-gp4-new.bin"
	arm-none-eabi-objcopy -Oihex /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/elec3300-f22-project-gp4-new.elf /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/elec3300-f22-project-gp4-new.hex
	arm-none-eabi-objcopy -Obinary /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/elec3300-f22-project-gp4-new.elf /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/elec3300-f22-project-gp4-new.bin

# Rule to build all files generated by this target.
CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/build: elec3300-f22-project-gp4-new.elf
.PHONY : CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/build

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/cmake_clean.cmake
.PHONY : CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/clean

CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/depend:
	cd /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug /Users/leonardyeung/Documents/comp-upstream-leonardyeung/ELEC/3300/elec3300-f22-project-gp4-new/cmake-build-debug/CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/elec3300-f22-project-gp4-new.elf.dir/depend

