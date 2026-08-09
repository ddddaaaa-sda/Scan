# CMake generated Testfile for 
# Source directory: /home/amov/scan_ws/src/SCAN-Planner-Pure-ROS2-master
# Build directory: /home/amov/scan_ws/build/scan_planner
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_test_scan_mid360_world_input_launch.py]=] "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/amov/scan_ws/build/scan_planner/test_results/scan_planner/test_test_scan_mid360_world_input_launch.py.xunit.xml" "--package-name" "scan_planner" "--output-file" "/home/amov/scan_ws/build/scan_planner/launch_test/test_test_scan_mid360_world_input_launch.py.txt" "--command" "/usr/bin/python3" "-m" "launch_testing.launch_test" "/home/amov/scan_ws/src/SCAN-Planner-Pure-ROS2-master/test/test_scan_mid360_world_input_launch.py" "--junit-xml=/home/amov/scan_ws/build/scan_planner/test_results/scan_planner/test_test_scan_mid360_world_input_launch.py.xunit.xml" "--package-name=scan_planner")
set_tests_properties([=[test_test_scan_mid360_world_input_launch.py]=] PROPERTIES  LABELS "launch_test" TIMEOUT "75" WORKING_DIRECTORY "/home/amov/scan_ws/build/scan_planner" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/launch_testing_ament_cmake/cmake/add_launch_test.cmake;131;ament_add_test;/home/amov/scan_ws/src/SCAN-Planner-Pure-ROS2-master/CMakeLists.txt;82;add_launch_test;/home/amov/scan_ws/src/SCAN-Planner-Pure-ROS2-master/CMakeLists.txt;0;")
subdirs("planner/plan_manage")
subdirs("planner/bspline_opt")
subdirs("planner/path_searching")
subdirs("planner/plan_env")
