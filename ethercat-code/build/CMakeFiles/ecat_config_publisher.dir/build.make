# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/abhishek/Documents/instrument-devel-app/ethercat-code

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/abhishek/Documents/instrument-devel-app/ethercat-code/build

# Include any dependencies generated for this target.
include CMakeFiles/ecat_config_publisher.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ecat_config_publisher.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ecat_config_publisher.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ecat_config_publisher.dir/flags.make

CMakeFiles/ecat_config_publisher.dir/master.cpp.o: CMakeFiles/ecat_config_publisher.dir/flags.make
CMakeFiles/ecat_config_publisher.dir/master.cpp.o: ../master.cpp
CMakeFiles/ecat_config_publisher.dir/master.cpp.o: CMakeFiles/ecat_config_publisher.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/abhishek/Documents/instrument-devel-app/ethercat-code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ecat_config_publisher.dir/master.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ecat_config_publisher.dir/master.cpp.o -MF CMakeFiles/ecat_config_publisher.dir/master.cpp.o.d -o CMakeFiles/ecat_config_publisher.dir/master.cpp.o -c /home/abhishek/Documents/instrument-devel-app/ethercat-code/master.cpp

CMakeFiles/ecat_config_publisher.dir/master.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ecat_config_publisher.dir/master.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/abhishek/Documents/instrument-devel-app/ethercat-code/master.cpp > CMakeFiles/ecat_config_publisher.dir/master.cpp.i

CMakeFiles/ecat_config_publisher.dir/master.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ecat_config_publisher.dir/master.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/abhishek/Documents/instrument-devel-app/ethercat-code/master.cpp -o CMakeFiles/ecat_config_publisher.dir/master.cpp.s

# Object files for target ecat_config_publisher
ecat_config_publisher_OBJECTS = \
"CMakeFiles/ecat_config_publisher.dir/master.cpp.o"

# External object files for target ecat_config_publisher
ecat_config_publisher_EXTERNAL_OBJECTS =

ecat_config_publisher: CMakeFiles/ecat_config_publisher.dir/master.cpp.o
ecat_config_publisher: CMakeFiles/ecat_config_publisher.dir/build.make
ecat_config_publisher: /usr/local/lib/libiceoryx_posh.a
ecat_config_publisher: /usr/local/lib/libethercat.so
ecat_config_publisher: /usr/local/lib/libiceoryx_dust.a
ecat_config_publisher: /usr/local/lib/libiceoryx_hoofs.a
ecat_config_publisher: /usr/local/lib/libiceoryx_platform.a
ecat_config_publisher: CMakeFiles/ecat_config_publisher.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/abhishek/Documents/instrument-devel-app/ethercat-code/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ecat_config_publisher"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ecat_config_publisher.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ecat_config_publisher.dir/build: ecat_config_publisher
.PHONY : CMakeFiles/ecat_config_publisher.dir/build

CMakeFiles/ecat_config_publisher.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ecat_config_publisher.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ecat_config_publisher.dir/clean

CMakeFiles/ecat_config_publisher.dir/depend:
	cd /home/abhishek/Documents/instrument-devel-app/ethercat-code/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/abhishek/Documents/instrument-devel-app/ethercat-code /home/abhishek/Documents/instrument-devel-app/ethercat-code /home/abhishek/Documents/instrument-devel-app/ethercat-code/build /home/abhishek/Documents/instrument-devel-app/ethercat-code/build /home/abhishek/Documents/instrument-devel-app/ethercat-code/build/CMakeFiles/ecat_config_publisher.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ecat_config_publisher.dir/depend

