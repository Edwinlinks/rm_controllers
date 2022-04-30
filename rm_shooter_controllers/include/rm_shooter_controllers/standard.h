/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by huakang on 2021/1/18.
//

#pragma once

#include <effort_controllers/joint_velocity_controller.h>
#include <effort_controllers/joint_position_controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <rm_common/ros_utilities.h>
#include <realtime_tools/realtime_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <rm_shooter_controllers/ShooterConfig.h>
#include <rm_msgs/ShootCmd.h>

#include <utility>

namespace rm_shooter_controllers
{
struct Config
{
  double block_effort, block_speed, block_duration, block_overtime, anti_block_angle, anti_block_threshold;
  double qd_10, qd_15, qd_16, qd_18, qd_30, lf_extra_rotat_speed;
};

class Controller : public controller_interface::MultiInterfaceController<hardware_interface::EffortJointInterface,
                                                                         rm_control::RobotStateInterface>
{
public:
  Controller() = default;
  /** @brief Get params from param server. Init the Joint_Velocity_Controller and Joint_position_Controller.
   *
   * Get params from param server and add parameters related to friction wheel's angular velocity corresponding
   * to each bullet speed and trigger block detection parameters to dynamic_reconfigure. Init the
   * right friction controller, left friction controller, and trigger wheel controller.
   *
   * @param robot_hw The robot hardware abstraction.
   * @param root_nh A NodeHandle in the root of the controller manager namespace. This is where the ROS interfaces are
   * setup (publishers, subscribers, services).
   * @param controller_nh A NodeHandle in the namespace of the controller. This is where the controller-specific
   * configuration resides.
   * @return True if the left and right friction wheel and trigger wheel controller init successfully, false when any
   * one of them initialize failed.
   */
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  /** @brief Execute the corresponding action according to the state of the controller.
   *
   * If the state switch to STOP, the right and left friction controller will set velocity of the joint to 0.
   * If the state switch to READY, shooter controller will execute normalize, function to reset the trigger
   * wheel to the correct position. If the state switch to PUSH, shooter controller will send commands
   * to rotate the trigger wheel. If the state switch to BLOCK, shooter controller reverse an angle to get
   * rid of blocking, and then conduct block detection.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void update(const ros::Time& time, const ros::Duration& period) override;
  /** @brief Switch the state of shooter controller to STOP, and set the value of state_changed_ to TRUE.
   *
   *
   *
   * @param time The current time.
   */
  void starting(const ros::Time& /*time*/) override;

private:
  /** @brief Set the velocity of the friction wheel to 0 to make it stop.
   *
   * When the state of shooter controller switch to STOP, the right and left friction controller will
   * be set velocity of the friction wheel to 0 to stop.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void stop(const ros::Time& time, const ros::Duration& period);
  /** @brief Execute @ref normalize to reset the trigger wheel to the correct position.
   *
   * When the state of controller switch to READY, shooter controller will execute @ref normalize
   * to reset the trigger wheel to the correct position.
   *
   * @param period The time passed since the last call to update.
   */
  void ready(const ros::Duration& period);
  /** @brief
   *
   * When the state of shooter controller switch to PUSH, shooter controller will judge
   * if the angular velocity of friction wheel is 0, when the angular velocity of friction wheel
   * is greater than 0, the controller will send commands to rotate the trigger wheel.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void push(const ros::Time& time, const ros::Duration& period);
  /** @brief The friction wheel will reverse anti_block_angle to try to get rid of blocking,
   * if reverse successfully shooter controller will enter PUSH state.
   *
   * When the state of shooter controller switch to BLOCK, the friction wheel will
   * reverse anti_block_angle to try to get rid of BLOCK state, and then detect whether
   * shooter controller still blocking, if get rid of blocking, shooter controller will enter PUSH state.
   *
   * @param time The current time.
   * @param period The time passed since the last call to update.
   */
  void block(const ros::Time& time, const ros::Duration& period);
  /** @brief Set the corresponding angular velocity for the left and right friction wheels according to values of cmd_.speed.
   *
   * The shooter controller will read the rm_msgs::ShootCmd date, and set the angular velocity for friction wheel
   * according to the bullet speed set by command.
   *
   * @param cmd Command message about the state of shooter controller, bullet speed, frequency of shooting.
   */
  void setSpeed(const rm_msgs::ShootCmd& cmd);
  /** @brief Before the trigger wheel is set command for rotating, set the trigger wheel to the correct position.
   *
   * Before the friction wheel is ready to enter PUSH state,
   *
   */
  void normalize();
  /** @brief Write the data of msg inside to the non real-time buffer.
   *
   *
   *
   * @param msg Message about the state of shooter controller, bullet speed, frequency of shooting.
   */
  void commandCB(const rm_msgs::ShootCmdConstPtr& msg)
  {
    cmd_rt_buffer_.writeFromNonRT(*msg);
  }
  /** @brief Call back function when the server gets a reconfiguration request it will call our
     * callback function to update the value in the parameter.
     *
     *
     *
     * @param config Date related to friction wheel's angular velocity corresponding
     * to each bullet speed and trigger block detection parameters.
     */
  void reconfigCB(rm_shooter_controllers::ShooterConfig& config, uint32_t /*level*/);

  hardware_interface::EffortJointInterface* effort_joint_interface_{};
  effort_controllers::JointVelocityController ctrl_friction_l_, ctrl_friction_r_;
  effort_controllers::JointPositionController ctrl_trigger_;
  int push_per_rotation_{};
  double push_qd_threshold_{};
  bool dynamic_reconfig_initialized_ = false;
  bool state_changed_ = false;
  bool maybe_block_ = false;

  ros::Time last_shoot_time_, block_time_, last_block_time_;
  // The state of shooter controller
  enum
  {
    STOP,
    READY,
    PUSH,
    BLOCK
  };
  int state_ = STOP;
  Config config_{};
  realtime_tools::RealtimeBuffer<Config> config_rt_buffer;
  realtime_tools::RealtimeBuffer<rm_msgs::ShootCmd> cmd_rt_buffer_;
  rm_msgs::ShootCmd cmd_;
  ros::Subscriber cmd_subscriber_;
  dynamic_reconfigure::Server<rm_shooter_controllers::ShooterConfig>* d_srv_{};
};

}  // namespace rm_shooter_controllers
