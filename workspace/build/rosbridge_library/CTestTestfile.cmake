# CMake generated Testfile for 
# Source directory: /home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library
# Build directory: /home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_ros_loader "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/test_results/rosbridge_library/test_ros_loader.xunit.xml" "--package-name" "rosbridge_library" "--output-file" "/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/ament_cmake_pytest/test_ros_loader.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library/test/internal/test_ros_loader.py" "-o" "cache_dir=/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/ament_cmake_pytest/test_ros_loader/.cache" "--junit-xml=/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/test_results/rosbridge_library/test_ros_loader.xunit.xml" "--junit-prefix=rosbridge_library")
set_tests_properties(test_ros_loader PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library/CMakeLists.txt;20;ament_add_pytest_test;/home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library/CMakeLists.txt;0;")
add_test(test_message_conversion "/usr/bin/python3.10" "-u" "/opt/ros/iron/share/ament_cmake_test/cmake/run_test.py" "/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/test_results/rosbridge_library/test_message_conversion.xunit.xml" "--package-name" "rosbridge_library" "--output-file" "/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/ament_cmake_pytest/test_message_conversion.txt" "--command" "/usr/bin/python3.10" "-u" "-m" "pytest" "/home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library/test/internal/test_message_conversion.py" "-o" "cache_dir=/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/ament_cmake_pytest/test_message_conversion/.cache" "--junit-xml=/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library/test_results/rosbridge_library/test_message_conversion.xunit.xml" "--junit-prefix=rosbridge_library")
set_tests_properties(test_message_conversion PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/home/jason/Desktop/STUDY/ART-Driver/workspace/build/rosbridge_library" _BACKTRACE_TRIPLES "/opt/ros/iron/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/iron/share/ament_cmake_pytest/cmake/ament_add_pytest_test.cmake;177;ament_add_test;/home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library/CMakeLists.txt;21;ament_add_pytest_test;/home/jason/Desktop/STUDY/ART-Driver/workspace/src/rosbridge_suite/rosbridge_library/CMakeLists.txt;0;")
