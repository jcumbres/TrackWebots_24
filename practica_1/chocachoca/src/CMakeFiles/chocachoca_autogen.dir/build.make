# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.29

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
CMAKE_COMMAND = /snap/clion/296/bin/cmake/linux/x64/bin/cmake

# The command to remove a file.
RM = /snap/clion/296/bin/cmake/linux/x64/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca

# Utility rule file for chocachoca_autogen.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/chocachoca_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/chocachoca_autogen.dir/progress.make

src/CMakeFiles/chocachoca_autogen: src/chocachoca_autogen/timestamp

src/chocachoca_autogen/timestamp: /usr/lib/qt6/libexec/moc
src/chocachoca_autogen/timestamp: /usr/lib/qt6/libexec/uic
src/chocachoca_autogen/timestamp: src/CMakeFiles/chocachoca_autogen.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold --progress-dir=/home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC and UIC for target chocachoca"
	cd /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src && /snap/clion/296/bin/cmake/linux/x64/bin/cmake -E cmake_autogen /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src/CMakeFiles/chocachoca_autogen.dir/AutogenInfo.json Debug
	cd /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src && /snap/clion/296/bin/cmake/linux/x64/bin/cmake -E touch /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src/chocachoca_autogen/timestamp

chocachoca_autogen: src/CMakeFiles/chocachoca_autogen
chocachoca_autogen: src/chocachoca_autogen/timestamp
chocachoca_autogen: src/CMakeFiles/chocachoca_autogen.dir/build.make
.PHONY : chocachoca_autogen

# Rule to build all files generated by this target.
src/CMakeFiles/chocachoca_autogen.dir/build: chocachoca_autogen
.PHONY : src/CMakeFiles/chocachoca_autogen.dir/build

src/CMakeFiles/chocachoca_autogen.dir/clean:
	cd /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src && $(CMAKE_COMMAND) -P CMakeFiles/chocachoca_autogen.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/chocachoca_autogen.dir/clean

src/CMakeFiles/chocachoca_autogen.dir/depend:
	cd /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src /home/robocomp/robocomp/components/grupo_12/Robotica_24-25/practica_1/chocachoca/src/CMakeFiles/chocachoca_autogen.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/CMakeFiles/chocachoca_autogen.dir/depend

