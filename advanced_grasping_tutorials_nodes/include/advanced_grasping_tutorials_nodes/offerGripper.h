#ifndef ADVANCED_GRASPING_TUTORIALS_OFFER_GRIPPER_H
#define ADVANCED_GRASPING_TUTORIALS_OFFER_GRIPPER_H

// Behaviortree
#include <behaviortree_ros_actions/behaviortree_pal_utils.h>
#include <pal_bt_actions_moveit/mtcnode.h>
#include <pal_bt_actions_moveit/mtc_stage_utils.h>

// MoveIt Task Constructor
#include <moveit/task_constructor/solvers/pipeline_planner.h>


typedef std::chrono::milliseconds Milliseconds;
typedef std::chrono::time_point<std::chrono::high_resolution_clock> Timepoint;

namespace pal
{
class MTCOfferGripperAction : public MTCNode
{
public:
  MTCOfferGripperAction(const std::string &name, const BT::NodeConfiguration &config);

  static BT::PortsList providedPorts()
  {
    static BT::PortsList ports = { BT::InputPort<std::string>("grasping_arm") };
    return ports;
  }

private:
  virtual moveit::task_constructor::Task setupTask() override;

  virtual BT::Optional<std::string> getGraspingArm()
  {
    return getInput<std::string>("grasping_arm");
  }

  ros::NodeHandle nh_;
  bool callbackresponse_;

  std::string grasping_arm_suffix_;
  std::string eef_frame_;
  std::string eef_group_name_;
  std::string arm_group_name_;
};

}  // namespace pal
#endif
