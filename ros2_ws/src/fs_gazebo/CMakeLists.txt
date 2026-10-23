cmake_minimum_required(VERSION 3.16)
project(fs_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

# Soil core: pure C++, shared by both systems and the unit tests.
add_library(fs_soil SHARED src/SoilMap.cc)
target_include_directories(fs_soil PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

set(fs_systems terramechanics:Terramechanics auger_soil:AugerSoil swerve_drive:SwerveDrive
  joint_state_rate:JointStateRate plug_inventory:PlugInventory)
foreach(entry ${fs_systems})
  string(REPLACE ":" ";" parts ${entry})
  list(GET parts 0 sys)
  list(GET parts 1 src)
  add_library(fs_${sys}_system SHARED src/${src}.cc)
  target_link_libraries(fs_${sys}_system fs_soil
    gz-sim::gz-sim gz-plugin::register gz-transport::gz-transport gz-msgs::gz-msgs)
  install(TARGETS fs_${sys}_system LIBRARY DESTINATION lib)
endforeach()

install(TARGETS fs_soil EXPORT export_fs_soil LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_sites.py DESTINATION lib/${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

# Gazebo finds the systems and models through these when the workspace is sourced.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/fs_gazebo.dsv.in")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_soil_model test/test_soil_model.cpp)
  target_link_libraries(test_soil_model fs_soil)
  target_compile_definitions(test_soil_model PRIVATE FS_SITES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/sites")
  ament_add_pytest_test(test_sites test/test_sites.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(fs_soil)
ament_package()
