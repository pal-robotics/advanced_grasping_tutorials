cmake_minimum_required(VERSION 3.0.2)
project(advanced_grasping_tutorials_nodes)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    pal_bt_actions_moveit
    
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  pal_bt_actions_moveit
  
)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/offerGripper.cpp
  src/behaviortree_register_example_plugins.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE  BT_PLUGIN_EXPORT)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
#############
## Install ##
#############



