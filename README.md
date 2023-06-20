# PAL Advanced Grasping Tutorial

This package contains a tutorial for the Advanced Grasping package from PAL Robotics. This package consists of a framework that simplifies the grasping capabilities of TIAGo and TIAGo++. In this tutorial it is explained which customization options are available for Advanced Grasping in order to adapt the framework to specific applications. The following tasks will be covered:

* Create a new behavior tree node
* Change a behavior tree node within a behaviortree
* Change the behavior tree linked to a specific action server
* Create a new action server plugin.

### Prerequisites

In order to compile this package you will need:

* A docker container of PAL Gallium with the package pal_bt_grasping
* The packages pal_bt_actions_moveit and pal_bt_grasping. Both are non public packages distributed by PAL Robotics.

### Installing

Open the docker container of Gallium and create a new [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Clone this repository on your workspace.

``` bash
git clone ...
```

And then compile it.

``` bash
catkin build
```

And finally source the workspace.

## Running the tutorial


Tutorials can run in a simulation environment or on a real robot.

To launch the simulation environment with the prerequisites for Advanced Grasping:
```bash
roslaunch tiago_gazebo tiago_gazebo.launch advanced_grasping:=true  end_effector:=pal-gripper
```

Copy the behaviortree files from the folder [```config/bt```](/advanced_grasping_tutorials/config/bt/) to ```/.pal/advanced_grasping/bt```. 

``` bash
roscd advanced_grasping_tutorials
cp config/bt/* /.pal/advanced_grasping/bt/
```

To launch the grasping server:
```bash
roslaunch advanced_grasping_tutorials advanced_grasping.launch
```
To launch the demo of the tutorial:
```bash
rosrun advanced_grasping_tutorials example_demo.py
```

In this simple example TIAGo will offer its arm, and tries to grasp the object that is placed in its gripper. 

## Create a new behavior tree node

This section will list the steps to create a new node that can be used for behavior trees. The code can be found in the package [advanced_grasping_tutorials_nodes](/advanced_grasping_tutorials_nodes/). Creating the node consists of three steps:

1. Create a new c++ class with header file. This class ```MTCOfferGripperAction``` is inherited from a behavior tree node class, in this case from the custom PAL class ```MTCNode```. This class allows to create behavior tree nodes executing tasks of [Moveit Task Constructor](https://github.com/ros-planning/moveit_task_constructor). With ```MTCOfferGripperAction``` TIAGo move its gripper in front of him. In this class it is important to set the function ```providedPorts```.  This functions sets the input and output ports that allows the node to retrieve and set valuables in the [blackboard](https://www.behaviortree.dev/docs/3.8/tutorial-basics/tutorial_02_basic_ports) of the behavior tree. In this case the node requires information on which side is need to grasp if TIAGo++ is used. For TIAGo this is not necessary.

``` cpp
  static BT::PortsList providedPorts()
  {
    static BT::PortsList ports = { BT::InputPort<std::string>("grasping_arm") };
    return ports;
  }
```
2. Register the newly created behavior tree node as a plugin that can be used by BehaviorTreeCPP. This is done in [behaviortree_register_example_plugins.cpp](/advanced_grasping_tutorials_nodes/src/behaviortree_register_example_plugins.cpp). The name of the node *OfferGripperAction* will be used to call the node from the behavior tree xml file.

``` cpp
  factory.registerNodeType<pal::MTCOfferGripperAction>("OfferGripperAction");

```

3. Export the library as a bahaviortree plugin in both the [CMakeLists.txt](/advanced_grasping_tutorials_nodes/CMakeLists.txt) and the [package.xml](/advanced_grasping_tutorials_nodes/package.xml). 

CMakeLists.txt
``` CMAKE
target_compile_definitions(${PROJECT_NAME} PRIVATE  BT_PLUGIN_EXPORT)

```
package.xml
``` xml
  <export>
    <behaviortree_cpp_v3 bt_lib_plugin="libadvanced_grasping_tutorials_nodes" />
  </export>
```

The new behavior tree node is now available to use in a behavior tree. In the next section will be shown how to change a specific node in a behavior tree.

## Change a behavior tree node

In this section will be explained how to change behavior tree nodes in the xml file of the tree. In this example the existing tree [example_tree.xml](/advanced_grasping_tutorials/config/bt/example_tree.xml) will be modified. In the original demo the robot would offer its gripper and grasp the object that is placed in its gripper. However, there is no feedback and even when there is no object the robot will retreat its arm. In this tutorial two new nodes are added. The GraspDetector node, a node that checks if the gripper has grasped an object and the isGrasped condition. This condition returns SUCCESS if an object is grasped and FAILURE otherwise. 

First copy the existing tree and name it grasp_detector.xml.
```bash
cp example_tree.xml grasp_detector.xml
```

In the new tree add the following line under the DisableHeadManager action:

```xml
<Action ID="GraspDetector" grasped="{grasp_status}" grasping_side="{grasping_side}"/>
```
The action has an input port, *grasping_side*, the side of the arm that is used for grasping, and an output port, *grasped*, a boolean that will be set as a variable in the blackboard with the name *grasp_status*.

Next add the update the line after the GraspEndEffectorAction with the following:
```xml
<Delay delay_msec="2000">
    <Action ID="GraspEndEffectorAction" grasping_side="{grasping_side}"/>
</Delay>    
<Delay delay_msec="250"> <!-- A small delay to allow the grasp detector information to be set in the blackboard -->
    <Condition ID="isGrasped" grasped="{grasp_status}"/>
</Delay>   
``` 
This will add a condition node that checks if an object has been grasped. If no object has been grasped the gripper will open and TIAGo tries to grasp again.

In the next section will be shown how to link this newly created tree to the Advanced Grasping package.


## Change the behavior tree

In this section will be explained how to change the behavior tree linked to an action of the Advanced Grasping package.

First, copy the behavior trees to the `.pal` folder again. Ensure that the newly created tree is now located in this folder.

``` bash
roscd advanced_grasping_tutorials
cp config/bt/* ~/.pal/bt/
```

Next, update the [example_server_config.yaml](/advanced_graspign_tutorials/config/example_server_config.yaml) file to link the new tree to the action `/example_grasp_action`.

``` yaml
  actions:
    - example_grasp_action:
        behaviortree_file: grasp_detector.xml
        action_plugin: advanced_grasping/ExampleServer
```

Now the behavior tree has been properly linked to the action. Run the demo again to test the new tree.

In the next section will be shown how a new action can be created for the Advanced Grasping package.

## Create a new action server plugin

In this section is explained how to create a new action server that can be used in the Advanced Grasping framework. Creating a new action server is necessary when the current actions are not sufficient, e.g. other action arguments are required for the behavior tree to run correctly.

In the package `advanced_grasping_tutorials` a new action server is created, *ExampleServer*. This server uses the existing action *GraspObjectAction*. A [cpp class](/advanced_grasping_tutorials/src/example_plugin.cpp) and corresponding [header file](/example_grasping_tutorials/include/advanced_grasping_tutorials/example_plugin.h) are created for this new action server. Note that the class needs to inherit from the *AdvancedGraspingServer* class, with an action as template argument.

For the new server two functions have to be implemented:
1. *configureBlackboard()*: This function sets the information of the goal arguments in the blackboard as variables before the behavior tree will start.
2. *setResult()*: After the behavior tree is finished this function will retrieve information of the blackboard and return it as the result of the action.

Next, the new class has to be registered as a plugin. More information about plugins can be found [here](https://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin). First, within the[ C++ class](/advanced_grasping_tutorials/src/example_plugin.cpp), the class has to be registered as plugin:

```cpp
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(advanced_grasping::ExampleServer, advanced_grasping::AdvancedGraspingPlugin)
```
Then a [plugin_description.xml](/advanced_grasping_tutorials/advanced_grasping_example_plugin_description.xml) is created. This description will tell the pluginloader in which library to find the new plugin. The name of the library is set in the CMakeLists.txt

```xml
<library path="libadvanced_grasping_tutorials">

  <class name="advanced_grasping/ExampleServer" type="advanced_grasping::ExampleServer" base_class_type="advanced_grasping::AdvancedGraspingPlugin">
    <description>
        Load an example grasping server for advanced grasping
    </description>
  </class>
</library>
```

Update the CMakeLists.txt and package.xml to export the plugin:
* CMakeLists.txt
```CMake
install(FILES
  advanced_grasping_example_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```
* package.xml
```xml
  <export>
    <pal_bt_grasping_tiago plugin="${prefix}/advanced_grasping_example_plugin_description.xml"/>
  </export>
```
And finally add the new action server with the name of the action and its linked tree to the [example_server_config.yaml](/advanced_grasping_tutorials/config/example_server_config.yaml).

```yaml
  actions:
    - example_grasp_action:
        behaviortree_file: example_tree.xml
        action_plugin: advanced_grasping/ExampleServer
```



## Authors

* **David ter kuile**
