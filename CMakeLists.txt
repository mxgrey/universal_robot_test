cmake_minimum_required(VERSION 2.8.3)

project(universal_robot_test)



find_package(catkin REQUIRED COMPONENTS
  ur_kinematics
)

catkin_package(
  CATKIN_DEPENDS
    ur_kinematics
)

add_executable(universal_robot_test src/test.cpp)
set_property(TARGET universal_robot_test PROPERTY CXX_STANDARD 11)

target_link_libraries(universal_robot_test PRIVATE
  ${ur_kinematics_LIBRARIES}
)

target_include_directories(universal_robot_test PRIVATE
  ${ur_kinematics_INCLUDE_DIRS}
)

install(TARGETS universal_robot_test
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
