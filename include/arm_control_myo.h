#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "ros_myo/MyoArm.h"
#include "ros_myo/MyoPose.h"
#include "ros_myo/EmgArray.h"
#include "tf/transform_datatypes.h"
#include "kinova_msgs/FingerPosition.h"
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/PoseVelocity.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

namespace IQR {
  class ArmControlMyo {

  public:
    ArmControlMyo(ros::NodeHandle& nod_);
    ros::Subscriber position_sub_, finger_sub_;
    ros::Publisher pose_vel_pub_, finger_pub_;
    double yaw_init, pitch_init, roll_init;
    float linear_vel_x, linear_vel_y, linear_vel_z;
    bool init = true, Cartesian_mode = true ,fetch_mode_ = false, fix_mode_ = false;
    int finger1_goal_ = 100, finger2_goal_ = 100, finger3_goal_ = 100;
  private:
    void myoImuCallback(const sensor_msgs::Imu::ConstPtr& imu);
    void myoGestCallback(const ros_myo::MyoPose::ConstPtr& pose);

  };
}