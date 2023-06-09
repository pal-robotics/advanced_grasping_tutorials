
#pragma once
#include <pal_bt_grasping_tiago/plugins/advanced_grasping_server.h>
#include <pal_bt_grasping_msgs/GraspObjectAction.h>


namespace advanced_grasping
{
class ExampleServer : public AdvancedGraspingServer<pal_bt_grasping_msgs::GraspObjectAction>
{
public:
  ExampleServer();

protected:
  bool configureBlackboard(const pal_bt_grasping_msgs::GraspObjectGoalConstPtr &goal,
                           BT::Blackboard::Ptr &blackboard) override;

  pal_bt_grasping_msgs::GraspObjectResult setResult(const bool success, const BT::Blackboard::Ptr &blackboard = nullptr, const std::string &error_msg = std::string("")) override;

};
}  // namespace advanced_grasping
