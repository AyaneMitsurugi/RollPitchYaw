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
CMAKE_SOURCE_DIR = /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build

# Include any dependencies generated for this target.
include CMakeFiles/RollPitchYaw.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/RollPitchYaw.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/RollPitchYaw.dir/flags.make

CMakeFiles/RollPitchYaw.dir/src/main.cpp.o: CMakeFiles/RollPitchYaw.dir/flags.make
CMakeFiles/RollPitchYaw.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/RollPitchYaw.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RollPitchYaw.dir/src/main.cpp.o -c /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/main.cpp

CMakeFiles/RollPitchYaw.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RollPitchYaw.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/main.cpp > CMakeFiles/RollPitchYaw.dir/src/main.cpp.i

CMakeFiles/RollPitchYaw.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RollPitchYaw.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/main.cpp -o CMakeFiles/RollPitchYaw.dir/src/main.cpp.s

CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.o: CMakeFiles/RollPitchYaw.dir/flags.make
CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.o: ../src/AHRS/Fusion/FusionAhrs.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.o   -c /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Fusion/FusionAhrs.c

CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Fusion/FusionAhrs.c > CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.i

CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Fusion/FusionAhrs.c -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.s

CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.o: CMakeFiles/RollPitchYaw.dir/flags.make
CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.o: ../src/AHRS/Fusion/FusionBias.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.o   -c /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Fusion/FusionBias.c

CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Fusion/FusionBias.c > CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.i

CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Fusion/FusionBias.c -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.s

CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.o: CMakeFiles/RollPitchYaw.dir/flags.make
CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.o: ../src/AHRS/Madgwick/MadgwickAHRS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.o -c /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Madgwick/MadgwickAHRS.cpp

CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Madgwick/MadgwickAHRS.cpp > CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.i

CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Madgwick/MadgwickAHRS.cpp -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.s

CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.o: CMakeFiles/RollPitchYaw.dir/flags.make
CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.o: ../src/AHRS/Mahony/MahonyAHRS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.o -c /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Mahony/MahonyAHRS.cpp

CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Mahony/MahonyAHRS.cpp > CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.i

CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Mahony/MahonyAHRS.cpp -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.s

CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.o: CMakeFiles/RollPitchYaw.dir/flags.make
CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.o: ../src/AHRS/Common/CommonAHRS.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.o -c /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Common/CommonAHRS.cpp

CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Common/CommonAHRS.cpp > CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.i

CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/src/AHRS/Common/CommonAHRS.cpp -o CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.s

# Object files for target RollPitchYaw
RollPitchYaw_OBJECTS = \
"CMakeFiles/RollPitchYaw.dir/src/main.cpp.o" \
"CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.o" \
"CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.o" \
"CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.o" \
"CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.o" \
"CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.o"

# External object files for target RollPitchYaw
RollPitchYaw_EXTERNAL_OBJECTS =

RollPitchYaw: CMakeFiles/RollPitchYaw.dir/src/main.cpp.o
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionAhrs.c.o
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/src/AHRS/Fusion/FusionBias.c.o
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/src/AHRS/Madgwick/MadgwickAHRS.cpp.o
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/src/AHRS/Mahony/MahonyAHRS.cpp.o
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/src/AHRS/Common/CommonAHRS.cpp.o
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/build.make
RollPitchYaw: CMakeFiles/RollPitchYaw.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX executable RollPitchYaw"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/RollPitchYaw.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/RollPitchYaw.dir/build: RollPitchYaw

.PHONY : CMakeFiles/RollPitchYaw.dir/build

CMakeFiles/RollPitchYaw.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/RollPitchYaw.dir/cmake_clean.cmake
.PHONY : CMakeFiles/RollPitchYaw.dir/clean

CMakeFiles/RollPitchYaw.dir/depend:
	cd /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build /home/ayane/Master_GitHub_RollPitchYaw/RollPitchYaw/build/CMakeFiles/RollPitchYaw.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/RollPitchYaw.dir/depend

