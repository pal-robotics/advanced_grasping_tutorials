cmake_minimum_required(VERSION 3.0.2)
project(advanced_grasping_tutorials)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    pal_bt_grasping_tiago
    advanced_grasping_tutorials_nodes
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  pal_bt_grasping_tiago
  advanced_grasping_tutorials_nodes
)

###########
## Build ##
###########
include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/example_plugin.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config launch scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(FILES
  advanced_grasping_example_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/example_demo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

