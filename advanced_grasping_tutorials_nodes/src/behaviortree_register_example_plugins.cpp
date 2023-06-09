#include "advanced_grasping_tutorials_nodes/offerGripper.h"

#include <behaviortree_cpp_v3/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pal::MTCOfferGripperAction>("OfferGripperAction");
}
