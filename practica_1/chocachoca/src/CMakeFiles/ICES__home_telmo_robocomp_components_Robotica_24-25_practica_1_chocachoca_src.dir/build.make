# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

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
CMAKE_SOURCE_DIR = /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca

# Utility rule file for ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.

# Include any custom commands dependencies for this target.
include src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/progress.make

ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src: src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "BU robocompdsl /home/telmo/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/CommonBehavior.ice"
	cd /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src && robocompdsl /home/telmo/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "BU robocompdsl /home/telmo/robocomp/interfaces/IDSLs/GenericBase.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/GenericBase.ice"
	cd /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src && robocompdsl /home/telmo/robocomp/interfaces/IDSLs/GenericBase.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/GenericBase.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "BU robocompdsl /home/telmo/robocomp/interfaces/IDSLs/Lidar3D.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/Lidar3D.ice"
	cd /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src && robocompdsl /home/telmo/robocomp/interfaces/IDSLs/Lidar3D.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/Lidar3D.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --blue --bold "BU robocompdsl /home/telmo/robocomp/interfaces/IDSLs/OmniRobot.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/OmniRobot.ice"
	cd /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src && robocompdsl /home/telmo/robocomp/interfaces/IDSLs/OmniRobot.idsl /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/OmniRobot.ice
.PHONY : ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src

# Rule to build all files generated by this target.
src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/build: ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src
.PHONY : src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/build

src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/clean:
	cd /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src && $(CMAKE_COMMAND) -P CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/clean

src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/depend:
	cd /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src /home/telmo/robocomp/components/Robotica_24-25/practica_1/chocachoca/src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : src/CMakeFiles/ICES__home_telmo_robocomp_components_Robotica_24-25_practica_1_chocachoca_src.dir/depend

