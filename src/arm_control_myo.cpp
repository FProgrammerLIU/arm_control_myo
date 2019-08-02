#include "arm_control_myo.h"

IQR::ArmControlMyo::ArmControlMyo(ros::NodeHandle& nod) {

  position_sub_ = nod.subscribe<sensor_msgs::Imu>("/myo_raw/myo_imu", 10, &IQR::ArmControlMyo::myoImuCallback, this);
  finger_sub_ = nod.subscribe<ros_myo::MyoPose>("/myo_raw/myo_gest", 10, &IQR::ArmControlMyo::myoGestCallback, this);
  pose_vel_pub_ = nod.advertise<kinova_msgs::PoseVelocity>("/j2s7s300_driver/in/cartesian_velocity", 1);

}

void IQR::ArmControlMyo::myoImuCallback(const sensor_msgs::Imu::ConstPtr& imu) {

  tf::Matrix3x3 mat(tf::Quaternion(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w));    
  double yaw, pitch, roll;    
  mat.getEulerYPR(yaw, pitch, roll);

  while(init) {
    yaw_init = yaw;
    pitch_init = pitch;
    roll_init = roll;
    init = false;
  }

  // ROS_INFO("%f %f %f", yaw_init, pitch_init, roll_init);
  // ROS_INFO("%f %f %f", yaw, pitch, roll);

  kinova_msgs::PoseVelocity pose_vel;
  if(!fetch_mode_) {
    if(!Cartesian_mode) {
      if(yaw-yaw_init >= 0.5) {
        pose_vel.twist_angular_z = -0.8;
      }
      else if(yaw-yaw_init <= -0.5) {
        pose_vel.twist_angular_z = 0.8;
      }
      if(pitch-pitch_init >= 0.5) {
        pose_vel.twist_angular_x = -0.4;
      }
      else if(pitch-pitch_init <= -0.5) {
        pose_vel.twist_angular_x = 0.4;
      }
      if(roll-roll_init >= 0.5 || roll-roll_init <= -5.8) {
        pose_vel.twist_angular_y = 0.4;
      }
      else if(roll-roll_init <= -0.5 || roll-roll_init >= 5.8) {
        pose_vel.twist_angular_y = -0.4;
      }
    }

    if (Cartesian_mode) {
      if(yaw-yaw_init >= 0.5) {
        pose_vel.twist_linear_y = 0.8;
      }
      else if(Cartesian_mode && yaw-yaw_init <= -0.5) {
        pose_vel.twist_linear_y = -0.8;
      }

      if(pitch-pitch_init >= 0.5) {
        pose_vel.twist_linear_x = 0.8;
      }
      else if(pitch-pitch_init <= -0.5) {
        pose_vel.twist_linear_x = -0.8;
      }

      if(roll-roll_init >= 0.5 || roll-roll_init <= -5.8) {
        pose_vel.twist_linear_z = -0.8;
      }
      else if(roll-roll_init <= -0.5 || roll-roll_init >= 5.8) {
        pose_vel.twist_linear_z = +0.8;
      }
    }
  pose_vel_pub_.publish(pose_vel); 
  }

  // if(fetch_mode_) {
  //   // if(first_start){
  //     actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac_finger("j2s7s300_driver/fingers_action/finger_positions", true);  
  //     ROS_INFO("Waiting for action server to start.");
  //     ac_finger.waitForServer();
  //     ROS_INFO("Action server started, sending goal.");
  //     first_start = false;
  //   // }    
  //   // if() {
  //     if(yaw-yaw_init >= 0.5 && finger1_goal_ <= 6400) {
  //       finger1_goal_ += 400;
  //       finger2_goal_ += 400;
  //       finger3_goal_ += 400;
  //       finger_goal.fingers.finger1 = finger1_goal_;
  //       finger_goal.fingers.finger2 = finger2_goal_;
  //       finger_goal.fingers.finger3 = finger3_goal_;
  //       // ac_finger.sendGoal(finger_goal);
  //       }
  //     else if(yaw-yaw_init <= -0.5 && finger1_goal_ >= 0) {
  //       finger1_goal_ -= 400;
  //       finger2_goal_ -= 400;
  //       finger3_goal_ -= 400;
  //       finger_goal.fingers.finger1 = finger1_goal_;
  //       finger_goal.fingers.finger2 = finger2_goal_;
  //       finger_goal.fingers.finger3 = finger3_goal_;
  //       // ac_finger.sendGoal(finger_goal);
  //     }
  //   // }  
  // }
}

void IQR::ArmControlMyo::myoGestCallback(const ros_myo::MyoPose::ConstPtr& myo_pose) {

  switch (myo_pose->pose) {

    case 1:
      fetch_mode_ = false;
      ROS_INFO("%s", "free_mode");
      break;
    case 2:
      fetch_mode_ = true;
      ROS_INFO("%s", "fetch_mode");

    if(fetch_mode_) {
  // if(first_start){
      kinova_msgs::SetFingersPositionGoal finger_goal;
      actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac_finger("j2s7s300_driver/fingers_action/finger_positions", true);  
      ROS_INFO("Waiting for action server to start.");
      ac_finger.waitForServer();
      ROS_INFO("Action server started, sending goal.");
      if(!fix_mode_ && finger1_goal_ <= 6400) {
        finger1_goal_ = 2800;
        finger2_goal_ = 2800;
        finger3_goal_ = 2800;
        finger_goal.fingers.finger1 = finger1_goal_;
        finger_goal.fingers.finger2 = finger2_goal_;
        finger_goal.fingers.finger3 = finger3_goal_;
        ac_finger.sendGoal(finger_goal);
        }
      else if(fix_mode_ && finger1_goal_ >= 100) {
        finger1_goal_ = 100;
        finger2_goal_ = 100;
        finger3_goal_ = 100;
        finger_goal.fingers.finger1 = finger1_goal_;
        finger_goal.fingers.finger2 = finger2_goal_;
        finger_goal.fingers.finger3 = finger3_goal_;
        ac_finger.sendGoal(finger_goal);
      }
  // }  
}

      // if (fist_mode == 2)
      // {
      //   finger_goal.fingers.finger1 = 500;
      //   finger_goal.fingers.finger2 = 500;
      //   finger_goal.fingers.finger3 = 500;
      //   ac_finger.sendGoal(finger_goal);
      //   fist_mode = 0;
      // }
      break;
    case 3:
      if(Cartesian_mode) {
        Cartesian_mode = false;
        ROS_INFO("%s", "drink_mode");
      } 
      else {
        Cartesian_mode = true;
        ROS_INFO("%s", "cartesian_mode");
      }
      break;
    case 4:
      if(fix_mode_) {
        fix_mode_ = false;
        ROS_INFO("%s", "fetch_mode");
      } 
      else {
        fix_mode_ = true;
        ROS_INFO("%s", "fix_mode_");
      }
      break;
  } 
    // ros::Rate loop_rate(500);

    // while (ros::ok() && fetch_mode_)
    // {
    //   kinova_msgs::SetFingersPositionGoal finger_goal;
    //   actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> ac_finger("j2s7s300_driver/fingers_action/finger_positions", true);  
    //   ROS_INFO("Waiting for action server to start.");
    //   ac_finger.waitForServer();
    //   ROS_INFO("Action server started, sending goal.");
    //   if(finger1_goal_ <= 6360 && !fix_mode_) {
    //     finger1_goal_ += 3;
    //     finger2_goal_ += 3;
    //     finger3_goal_ += 3;
    //     finger_goal.fingers.finger1 = finger1_goal_;
    //     finger_goal.fingers.finger2 = finger2_goal_;
    //     finger_goal.fingers.finger3 = finger3_goal_;
    //     ac_finger.sendGoal(finger_goal);
    //     }
    //   if(finger1_goal_ >= 40 && fix_mode_) {
    //     finger1_goal_ -= 3;
    //     finger2_goal_ -= 3;
    //     finger3_goal_ -= 3;
    //     finger_goal.fingers.finger1 = finger1_goal_;
    //     finger_goal.fingers.finger2 = finger2_goal_;
    //     finger_goal.fingers.finger3 = finger3_goal_;
    //   }
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }

  
}
