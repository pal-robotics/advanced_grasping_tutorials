#include "advanced_grasping_tutorials_nodes/offerGripper.h"

namespace pal
{
MTCOfferGripperAction::MTCOfferGripperAction(const std::string &name, const BT::NodeConfiguration &config)
  : MTCNode(name, config), grasping_arm_suffix_(""), nh_()
{
}


moveit::task_constructor::Task MTCOfferGripperAction::setupTask()
{
  moveit::task_constructor::Task t;

  // Get grasping side
  GET_INPUT(std::string, grasping_arm);
  if (!grasping_arm.value().empty())
  {
    grasping_arm_suffix_ = "_" + grasping_arm.value();
  }

  bool eef_available = grasp_utils::getEndEffector(grasping_arm.value(), eef_frame_,
                                                   eef_group_name_, arm_group_name_);

  // Sampling planner
  auto sampling_planner =
      std::make_shared<moveit::task_constructor::solvers::PipelinePlanner>();
  sampling_planner->setProperty("goal_joint_tolerance", 1e-5);

  t.loadRobotModel();


  /****************************************************
   *                                                  *
   *               Current State                      *
   *                                                  *
   ***************************************************/

  moveit::task_constructor::Stage *current_state_ptr =
      nullptr;  // Forward current_state on to grasp pose generator
  {
    auto current_state =
        std::make_unique<moveit::task_constructor::stages::CurrentState>("current state");
    current_state_ptr = current_state.get();
    t.add(std::move(current_state));
  }

  /****************************************************
   *                                                  *
   *                   Offer grippers                 *
   *                                                  *
   ***************************************************/
  {
    std::string motion_name = "offer";
    std::string planning_goal = motion_name + grasping_arm_suffix_;
    std::string stage_name = "Perform motion: " + motion_name;

    std::map<std::string, double> joints = grasp_utils::getPlanningGoal(planning_goal);

    auto stage = mtc_stage_utils::moveTo(arm_group_name_, joints, stage_name, sampling_planner);
    t.add(std::move(stage));
  }

  return t;
}



}  // namespace pal
