cmake_minimum_required(VERSION 2.8.3)
project(scanse_sweep)

find_package(catkin REQUIRED
  COMPONENTS
    pcl_conversions
    rosconsole
    roscpp
    sensor_msgs
)

find_package(PCL REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

include_directories(include SYSTEM ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS pcl_conversions rosconsole roscpp sensor_msgs DEPENDS PCL)

add_library(${PROJECT_NAME}
            include/${PROJECT_NAME}/error.hpp
            include/${PROJECT_NAME}/protocol.hpp
            include/${PROJECT_NAME}/queue.hpp
            include/${PROJECT_NAME}/serial.hpp
            include/${PROJECT_NAME}/sweep.h
            include/${PROJECT_NAME}/sweep.hpp
            src/${PROJECT_NAME}/protocol.cpp
            src/${PROJECT_NAME}/serial.cpp
            src/${PROJECT_NAME}/sweep.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(sweep_node src/sweep_node.cpp)
add_dependencies(sweep_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sweep_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(sweep_ctrl src/sweep_ctrl.cpp)
add_dependencies(sweep_ctrl ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sweep_ctrl ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

## Mark library for installation
install(TARGETS ${PROJECT_NAME} sweep_node sweep_ctrl
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)
## Mark launch files for installation
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)
## Mark RViz config files for installation
install(DIRECTORY rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
    PATTERN ".svn" EXCLUDE)
