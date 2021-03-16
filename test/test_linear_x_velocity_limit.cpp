
#include "test_common.h"

TEST_F(StandardChassisTest, testLinearXDirectionVelocityLimits) {
// wait for ROS
  while (!isControllerAlive()) {
    ros::Duration(0.1).sleep();
  }
// zero everything before test
  geometry_msgs::Twist cmd_vel{};
  rm_msgs::ChassisCmd cmd_chassis{};

  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;

  cmd_chassis.mode = cmd_chassis.RAW;
  cmd_chassis.effort_limit = 99;
  cmd_chassis.accel.linear.x = 10;
  cmd_chassis.accel.linear.y = 10;

  publish(cmd_chassis, cmd_vel);

//test X direction
  ros::Duration(2.0).sleep();
// get initial odom
  nav_msgs::Odometry old_odom = getLastOdom();
// send a big command
  cmd_vel.linear.x = 10.0;
  cmd_chassis.accel.linear.x = 1.0;
  publish(cmd_chassis, cmd_vel);
// wait for a while
  ros::Duration(3.5).sleep();

  nav_msgs::Odometry new_odom = getLastOdom();

// check if the robot speed is now 1.0 m.s-1, the limit
  EXPECT_LT(fabs(new_odom.twist.twist.linear.x - old_odom.twist.twist.linear.x), 3.8 + VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.twist.twist.angular.z - old_odom.twist.twist.angular.z), VELOCITY_TOLERANCE);
  EXPECT_LT(fabs(new_odom.pose.pose.position.y - old_odom.pose.pose.position.y), POSITION_TOLERANCE);

  cmd_vel.linear.x = 0.0;
  cmd_chassis.accel.linear.x = 0.0;
  publish(cmd_chassis, cmd_vel);

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_linear_x_direction_velocity_limits");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Duration(0.5).sleep();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}