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
CMAKE_SOURCE_DIR = /home/dennn/3dscaner

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dennn/3dscaner

# Include any dependencies generated for this target.
include CMakeFiles/scan.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/scan.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/scan.dir/flags.make

CMakeFiles/scan.dir/scan.cpp.o: CMakeFiles/scan.dir/flags.make
CMakeFiles/scan.dir/scan.cpp.o: scan.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dennn/3dscaner/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/scan.dir/scan.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/scan.dir/scan.cpp.o -c /home/dennn/3dscaner/scan.cpp

CMakeFiles/scan.dir/scan.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/scan.dir/scan.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dennn/3dscaner/scan.cpp > CMakeFiles/scan.dir/scan.cpp.i

CMakeFiles/scan.dir/scan.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/scan.dir/scan.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dennn/3dscaner/scan.cpp -o CMakeFiles/scan.dir/scan.cpp.s

CMakeFiles/scan.dir/scan.cpp.o.requires:
.PHONY : CMakeFiles/scan.dir/scan.cpp.o.requires

CMakeFiles/scan.dir/scan.cpp.o.provides: CMakeFiles/scan.dir/scan.cpp.o.requires
	$(MAKE) -f CMakeFiles/scan.dir/build.make CMakeFiles/scan.dir/scan.cpp.o.provides.build
.PHONY : CMakeFiles/scan.dir/scan.cpp.o.provides

CMakeFiles/scan.dir/scan.cpp.o.provides.build: CMakeFiles/scan.dir/scan.cpp.o

# Object files for target scan
scan_OBJECTS = \
"CMakeFiles/scan.dir/scan.cpp.o"

# External object files for target scan
scan_EXTERNAL_OBJECTS =

scan: CMakeFiles/scan.dir/scan.cpp.o
scan: CMakeFiles/scan.dir/build.make
scan: libscanlib.a
scan: libarduino_driver.a
scan: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libboost_system.so
scan: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
scan: /usr/lib/x86_64-linux-gnu/libboost_thread.so
scan: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
scan: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
scan: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
scan: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
scan: /usr/lib/x86_64-linux-gnu/libpthread.so
scan: /usr/lib/libpcl_common.so
scan: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
scan: /usr/lib/libpcl_kdtree.so
scan: /usr/lib/libpcl_octree.so
scan: /usr/lib/libpcl_search.so
scan: /usr/lib/x86_64-linux-gnu/libqhull.so
scan: /usr/lib/libpcl_surface.so
scan: /usr/lib/libpcl_sample_consensus.so
scan: /usr/lib/libOpenNI.so
scan: /usr/lib/libOpenNI2.so
scan: /usr/lib/libvtkCommon.so.5.8.0
scan: /usr/lib/libvtkFiltering.so.5.8.0
scan: /usr/lib/libvtkImaging.so.5.8.0
scan: /usr/lib/libvtkGraphics.so.5.8.0
scan: /usr/lib/libvtkGenericFiltering.so.5.8.0
scan: /usr/lib/libvtkIO.so.5.8.0
scan: /usr/lib/libvtkRendering.so.5.8.0
scan: /usr/lib/libvtkVolumeRendering.so.5.8.0
scan: /usr/lib/libvtkHybrid.so.5.8.0
scan: /usr/lib/libvtkWidgets.so.5.8.0
scan: /usr/lib/libvtkParallel.so.5.8.0
scan: /usr/lib/libvtkInfovis.so.5.8.0
scan: /usr/lib/libvtkGeovis.so.5.8.0
scan: /usr/lib/libvtkViews.so.5.8.0
scan: /usr/lib/libvtkCharts.so.5.8.0
scan: /usr/lib/libpcl_io.so
scan: /usr/lib/libpcl_filters.so
scan: /usr/lib/libpcl_features.so
scan: /usr/lib/libpcl_keypoints.so
scan: /usr/lib/libpcl_registration.so
scan: /usr/lib/libpcl_segmentation.so
scan: /usr/lib/libpcl_recognition.so
scan: /usr/lib/libpcl_visualization.so
scan: /usr/lib/libpcl_people.so
scan: /usr/lib/libpcl_outofcore.so
scan: /usr/lib/libpcl_tracking.so
scan: /usr/lib/libpcl_apps.so
scan: /usr/lib/x86_64-linux-gnu/libboost_system.so
scan: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
scan: /usr/lib/x86_64-linux-gnu/libboost_thread.so
scan: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
scan: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
scan: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
scan: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
scan: /usr/lib/x86_64-linux-gnu/libpthread.so
scan: /usr/lib/x86_64-linux-gnu/libqhull.so
scan: /usr/lib/libOpenNI.so
scan: /usr/lib/libOpenNI2.so
scan: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
scan: /usr/lib/libvtkCommon.so.5.8.0
scan: /usr/lib/libvtkFiltering.so.5.8.0
scan: /usr/lib/libvtkImaging.so.5.8.0
scan: /usr/lib/libvtkGraphics.so.5.8.0
scan: /usr/lib/libvtkGenericFiltering.so.5.8.0
scan: /usr/lib/libvtkIO.so.5.8.0
scan: /usr/lib/libvtkRendering.so.5.8.0
scan: /usr/lib/libvtkVolumeRendering.so.5.8.0
scan: /usr/lib/libvtkHybrid.so.5.8.0
scan: /usr/lib/libvtkWidgets.so.5.8.0
scan: /usr/lib/libvtkParallel.so.5.8.0
scan: /usr/lib/libvtkInfovis.so.5.8.0
scan: /usr/lib/libvtkGeovis.so.5.8.0
scan: /usr/lib/libvtkViews.so.5.8.0
scan: /usr/lib/libvtkCharts.so.5.8.0
scan: /usr/local/lib/libinifile++.so
scan: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
scan: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
scan: /usr/lib/libpcl_common.so
scan: /usr/lib/libpcl_kdtree.so
scan: /usr/lib/libpcl_octree.so
scan: /usr/lib/libpcl_search.so
scan: /usr/lib/libpcl_surface.so
scan: /usr/lib/libpcl_sample_consensus.so
scan: /usr/lib/libpcl_io.so
scan: /usr/lib/libpcl_filters.so
scan: /usr/lib/libpcl_features.so
scan: /usr/lib/libpcl_keypoints.so
scan: /usr/lib/libpcl_registration.so
scan: /usr/lib/libpcl_segmentation.so
scan: /usr/lib/libpcl_recognition.so
scan: /usr/lib/libpcl_visualization.so
scan: /usr/lib/libpcl_people.so
scan: /usr/lib/libpcl_outofcore.so
scan: /usr/lib/libpcl_tracking.so
scan: /usr/lib/libpcl_apps.so
scan: /usr/local/lib/libinifile++.so
scan: /usr/lib/libvtkViews.so.5.8.0
scan: /usr/lib/libvtkInfovis.so.5.8.0
scan: /usr/lib/libvtkWidgets.so.5.8.0
scan: /usr/lib/libvtkVolumeRendering.so.5.8.0
scan: /usr/lib/libvtkHybrid.so.5.8.0
scan: /usr/lib/libvtkParallel.so.5.8.0
scan: /usr/lib/libvtkRendering.so.5.8.0
scan: /usr/lib/libvtkImaging.so.5.8.0
scan: /usr/lib/libvtkGraphics.so.5.8.0
scan: /usr/lib/libvtkIO.so.5.8.0
scan: /usr/lib/libvtkFiltering.so.5.8.0
scan: /usr/lib/libvtkCommon.so.5.8.0
scan: /usr/lib/libvtksys.so.5.8.0
scan: CMakeFiles/scan.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable scan"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/scan.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/scan.dir/build: scan
.PHONY : CMakeFiles/scan.dir/build

CMakeFiles/scan.dir/requires: CMakeFiles/scan.dir/scan.cpp.o.requires
.PHONY : CMakeFiles/scan.dir/requires

CMakeFiles/scan.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/scan.dir/cmake_clean.cmake
.PHONY : CMakeFiles/scan.dir/clean

CMakeFiles/scan.dir/depend:
	cd /home/dennn/3dscaner && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dennn/3dscaner /home/dennn/3dscaner /home/dennn/3dscaner /home/dennn/3dscaner /home/dennn/3dscaner/CMakeFiles/scan.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/scan.dir/depend

