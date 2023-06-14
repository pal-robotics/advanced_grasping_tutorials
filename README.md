# PAL Advanced Grasping Tutorial

This package contains a tutorial for the Advanced Grasping package from PAL Robotics. This package consists of a framework that simplifies the grasping capabilities of TIAGo and TIAGo++. In this tutorial it is explained which customization options are available for Advanced Grasping in order to adapt the framework to specific applications. The following tasks will be covered:

* Create a new behavior tree node
* Change a behavior tree node within a behaviortree
* Change the behavior tree linked to a specific action server
* Create a new action server plugin.

### Prerequisites

In order to compile this package you will need:

* A docker container of PAL Gallium with the package pal_bt_grasping
* The packages pal_bt_actions_moveit and pal_bt_grasping. Both non public packages distributed by PAL Robotics.

### Installing

Open the docker container of Gallium. Create a new [workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

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


The tutorials can both be run in a simulation environment or on the real robot.

To launch the simulation environment with the prerequisites for Advanced Grasping:
```bash
roslaunch tiago_gazebo tiago_gazebo.launch advanced_grasping:=true
```

Copy the behaviortree files from the folder [```config/bt```](/advanced_grasping_tutorials/config/bt/) to ```~/.pal/bt```. 

``` bash
roscd advanced_grasping_tutorials
cp config/bt/* ~/pal/bt/
```

To launch the grasping server:
```bash
roslaunch advanced_grasping_tutorials advanced_grasping.launch
```
To launch the demo of the tutorial:
```bash
rosrun advanced_grasping_tutorials example_demo.py
```

## Create a new behavior tree node

The package [advanced_grasping_tutorials_nodes](/advanced_grasping_tutorials_nodes/) contains a new behavior tree. Creating one consists of three steps:

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

## Change a behavior tree node

## Change the behavior tree

## Create a new action server plugin




## Authors

* **David ter kuile**
