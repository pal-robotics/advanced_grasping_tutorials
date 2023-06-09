#include <advanced_grasping_tutorials/example_plugin.h>

namespace advanced_grasping
{
ExampleServer::ExampleServer()
  : AdvancedGraspingServer<pal_bt_grasping_msgs::GraspObjectAction>()
{
}


bool ExampleServer::configureBlackboard(const pal_bt_grasping_msgs::GraspObjectGoalConstPtr &goal,
                                      BT::Blackboard::Ptr &blackboard)
{
  // Setup blackboard
  std::string grasping_side =
      advanced_grasping::utils::checkGraspingSide(goal->grasping_side, robot_type_);
  blackboard->set("grasping_side", grasping_side);
  return true;
}

pal_bt_grasping_msgs::GraspObjectResult ExampleServer::setResult(const bool success, const BT::Blackboard::Ptr &blackboard, const std::string &error_msg)
{
  pal_bt_grasping_msgs::GraspObjectResult result;

  result.success = success;
  result.error_msg = error_msg;
  
  return result;
}

}  // namespace advanced_grasping


#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(advanced_grasping::ExampleServer, advanced_grasping::AdvancedGraspingPlugin)
