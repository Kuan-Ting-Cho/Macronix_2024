# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/airobots/node/src/Motor_Driver/src/MotorUnion

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/airobots/node/src/Motor_Driver/src/MotorUnion/build

# Include any dependencies generated for this target.
include Motor/CMakeFiles/Motor.dir/depend.make

# Include the progress variables for this target.
include Motor/CMakeFiles/Motor.dir/progress.make

# Include the compile flags for this target's objects.
include Motor/CMakeFiles/Motor.dir/flags.make

Motor/CMakeFiles/Motor.dir/Motor.cpp.o: Motor/CMakeFiles/Motor.dir/flags.make
Motor/CMakeFiles/Motor.dir/Motor.cpp.o: ../Motor/Motor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/airobots/node/src/Motor_Driver/src/MotorUnion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Motor/CMakeFiles/Motor.dir/Motor.cpp.o"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Motor.dir/Motor.cpp.o -c /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/Motor.cpp

Motor/CMakeFiles/Motor.dir/Motor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Motor.dir/Motor.cpp.i"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/Motor.cpp > CMakeFiles/Motor.dir/Motor.cpp.i

Motor/CMakeFiles/Motor.dir/Motor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Motor.dir/Motor.cpp.s"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/Motor.cpp -o CMakeFiles/Motor.dir/Motor.cpp.s

Motor/CMakeFiles/Motor.dir/MotorXm.cpp.o: Motor/CMakeFiles/Motor.dir/flags.make
Motor/CMakeFiles/Motor.dir/MotorXm.cpp.o: ../Motor/MotorXm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/airobots/node/src/Motor_Driver/src/MotorUnion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object Motor/CMakeFiles/Motor.dir/MotorXm.cpp.o"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Motor.dir/MotorXm.cpp.o -c /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorXm.cpp

Motor/CMakeFiles/Motor.dir/MotorXm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Motor.dir/MotorXm.cpp.i"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorXm.cpp > CMakeFiles/Motor.dir/MotorXm.cpp.i

Motor/CMakeFiles/Motor.dir/MotorXm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Motor.dir/MotorXm.cpp.s"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorXm.cpp -o CMakeFiles/Motor.dir/MotorXm.cpp.s

Motor/CMakeFiles/Motor.dir/MotorPro.cpp.o: Motor/CMakeFiles/Motor.dir/flags.make
Motor/CMakeFiles/Motor.dir/MotorPro.cpp.o: ../Motor/MotorPro.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/airobots/node/src/Motor_Driver/src/MotorUnion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object Motor/CMakeFiles/Motor.dir/MotorPro.cpp.o"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Motor.dir/MotorPro.cpp.o -c /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorPro.cpp

Motor/CMakeFiles/Motor.dir/MotorPro.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Motor.dir/MotorPro.cpp.i"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorPro.cpp > CMakeFiles/Motor.dir/MotorPro.cpp.i

Motor/CMakeFiles/Motor.dir/MotorPro.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Motor.dir/MotorPro.cpp.s"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorPro.cpp -o CMakeFiles/Motor.dir/MotorPro.cpp.s

Motor/CMakeFiles/Motor.dir/MotorProPlus.cpp.o: Motor/CMakeFiles/Motor.dir/flags.make
Motor/CMakeFiles/Motor.dir/MotorProPlus.cpp.o: ../Motor/MotorProPlus.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/airobots/node/src/Motor_Driver/src/MotorUnion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object Motor/CMakeFiles/Motor.dir/MotorProPlus.cpp.o"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Motor.dir/MotorProPlus.cpp.o -c /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorProPlus.cpp

Motor/CMakeFiles/Motor.dir/MotorProPlus.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Motor.dir/MotorProPlus.cpp.i"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorProPlus.cpp > CMakeFiles/Motor.dir/MotorProPlus.cpp.i

Motor/CMakeFiles/Motor.dir/MotorProPlus.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Motor.dir/MotorProPlus.cpp.s"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorProPlus.cpp -o CMakeFiles/Motor.dir/MotorProPlus.cpp.s

Motor/CMakeFiles/Motor.dir/MotorMx.cpp.o: Motor/CMakeFiles/Motor.dir/flags.make
Motor/CMakeFiles/Motor.dir/MotorMx.cpp.o: ../Motor/MotorMx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/airobots/node/src/Motor_Driver/src/MotorUnion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object Motor/CMakeFiles/Motor.dir/MotorMx.cpp.o"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Motor.dir/MotorMx.cpp.o -c /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorMx.cpp

Motor/CMakeFiles/Motor.dir/MotorMx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Motor.dir/MotorMx.cpp.i"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorMx.cpp > CMakeFiles/Motor.dir/MotorMx.cpp.i

Motor/CMakeFiles/Motor.dir/MotorMx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Motor.dir/MotorMx.cpp.s"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor/MotorMx.cpp -o CMakeFiles/Motor.dir/MotorMx.cpp.s

# Object files for target Motor
Motor_OBJECTS = \
"CMakeFiles/Motor.dir/Motor.cpp.o" \
"CMakeFiles/Motor.dir/MotorXm.cpp.o" \
"CMakeFiles/Motor.dir/MotorPro.cpp.o" \
"CMakeFiles/Motor.dir/MotorProPlus.cpp.o" \
"CMakeFiles/Motor.dir/MotorMx.cpp.o"

# External object files for target Motor
Motor_EXTERNAL_OBJECTS =

Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/Motor.cpp.o
Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/MotorXm.cpp.o
Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/MotorPro.cpp.o
Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/MotorProPlus.cpp.o
Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/MotorMx.cpp.o
Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/build.make
Motor/libMotor.so: Motor/dynamixel/libDynamixelSdk.so
Motor/libMotor.so: Motor/motor/libmotor.so
Motor/libMotor.so: Motor/CMakeFiles/Motor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/airobots/node/src/Motor_Driver/src/MotorUnion/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX shared library libMotor.so"
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Motor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Motor/CMakeFiles/Motor.dir/build: Motor/libMotor.so

.PHONY : Motor/CMakeFiles/Motor.dir/build

Motor/CMakeFiles/Motor.dir/clean:
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor && $(CMAKE_COMMAND) -P CMakeFiles/Motor.dir/cmake_clean.cmake
.PHONY : Motor/CMakeFiles/Motor.dir/clean

Motor/CMakeFiles/Motor.dir/depend:
	cd /home/airobots/node/src/Motor_Driver/src/MotorUnion/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/airobots/node/src/Motor_Driver/src/MotorUnion /home/airobots/node/src/Motor_Driver/src/MotorUnion/Motor /home/airobots/node/src/Motor_Driver/src/MotorUnion/build /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor /home/airobots/node/src/Motor_Driver/src/MotorUnion/build/Motor/CMakeFiles/Motor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Motor/CMakeFiles/Motor.dir/depend
