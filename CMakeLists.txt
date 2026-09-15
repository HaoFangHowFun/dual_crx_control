cmake_minimum_required(VERSION 3.8)
project(dual_crx_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR src/${PROJECT_NAME})

install(PROGRAMS
  scripts/dual_test_5_cartesion_sychro_motion.py
  scripts/dual_test_6_cartesian_circle_motion.py
  scripts/dual_test_7_cartesian_facing_circle_motion.py
  scripts/search_facing_circle.py
  scripts/dual_test2_periodic.py
  scripts/record_latency.py
  scripts/analyze_latency.py
  scripts/analyze_latency_trace.py
  scripts/dual_mock_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(cartesian tests/test_cartesian.py
    ENV PYTHONDONTWRITEBYTECODE=1 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    TIMEOUT 60
  )
  ament_add_pytest_test(interpolation tests/test_interpolation.py TIMEOUT 30)
  ament_add_pytest_test(circle tests/test_circle.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST TIMEOUT 60)
  ament_add_pytest_test(facing_circle tests/test_facing_circle.py TIMEOUT 60)
  ament_add_pytest_test(latency tests/test_latency.py
    ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST OPENBLAS_NUM_THREADS=1 TIMEOUT 60)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    urdf
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
