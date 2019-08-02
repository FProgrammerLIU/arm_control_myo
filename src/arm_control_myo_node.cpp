#include "arm_control_myo.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "arm_contorller");
  ros::NodeHandle nod_("~");
  IQR::ArmControlMyo armControlMyo(nod_);
  ros::spin();
  return 0;
}