# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/reto/projects/einbein

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reto/projects/einbein/build-arm

# Include any dependencies generated for this target.
include src/Test/Regelung/Controller/CMakeFiles/Controller.dir/depend.make

# Include the progress variables for this target.
include src/Test/Regelung/Controller/CMakeFiles/Controller.dir/progress.make

# Include the compile flags for this target's objects.
include src/Test/Regelung/Controller/CMakeFiles/Controller.dir/flags.make

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/flags.make
src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o: ../src/Test/Regelung/Controller/mainController.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/mainController.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Controller/mainController.cpp

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/mainController.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Controller/mainController.cpp > CMakeFiles/Controller.dir/mainController.cpp.i

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/mainController.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Controller/mainController.cpp -o CMakeFiles/Controller.dir/mainController.cpp.s

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.requires:
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.requires

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.provides: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/Controller/CMakeFiles/Controller.dir/build.make src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.provides.build
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.provides

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.provides.build: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/flags.make
src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o: ../src/Test/Regelung/Controller/Controller.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/Controller.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Controller/Controller.cpp

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/Controller.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Controller/Controller.cpp > CMakeFiles/Controller.dir/Controller.cpp.i

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/Controller.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Controller/Controller.cpp -o CMakeFiles/Controller.dir/Controller.cpp.s

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.requires:
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.requires

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.provides: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/Controller/CMakeFiles/Controller.dir/build.make src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.provides.build
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.provides

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.provides.build: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/flags.make
src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o: ../src/Test/Regelung/Controller/CSController.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controller.dir/CSController.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Controller/CSController.cpp

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controller.dir/CSController.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Controller/CSController.cpp > CMakeFiles/Controller.dir/CSController.cpp.i

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controller.dir/CSController.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Controller/CSController.cpp -o CMakeFiles/Controller.dir/CSController.cpp.s

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.requires:
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.requires

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.provides: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/Controller/CMakeFiles/Controller.dir/build.make src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.provides.build
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.provides

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.provides.build: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o

# Object files for target Controller
Controller_OBJECTS = \
"CMakeFiles/Controller.dir/mainController.cpp.o" \
"CMakeFiles/Controller.dir/Controller.cpp.o" \
"CMakeFiles/Controller.dir/CSController.cpp.o"

# External object files for target Controller
Controller_EXTERNAL_OBJECTS =

src/Test/Regelung/Controller/Controller: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o
src/Test/Regelung/Controller/Controller: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o
src/Test/Regelung/Controller/Controller: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o
src/Test/Regelung/Controller/Controller: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/build.make
src/Test/Regelung/Controller/Controller: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Controller"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Controller.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/Test/Regelung/Controller/CMakeFiles/Controller.dir/build: src/Test/Regelung/Controller/Controller
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/build

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/requires: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/mainController.cpp.o.requires
src/Test/Regelung/Controller/CMakeFiles/Controller.dir/requires: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/Controller.cpp.o.requires
src/Test/Regelung/Controller/CMakeFiles/Controller.dir/requires: src/Test/Regelung/Controller/CMakeFiles/Controller.dir/CSController.cpp.o.requires
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/requires

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/clean:
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller && $(CMAKE_COMMAND) -P CMakeFiles/Controller.dir/cmake_clean.cmake
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/clean

src/Test/Regelung/Controller/CMakeFiles/Controller.dir/depend:
	cd /home/reto/projects/einbein/build-arm && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reto/projects/einbein /home/reto/projects/einbein/src/Test/Regelung/Controller /home/reto/projects/einbein/build-arm /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller /home/reto/projects/einbein/build-arm/src/Test/Regelung/Controller/CMakeFiles/Controller.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Test/Regelung/Controller/CMakeFiles/Controller.dir/depend

