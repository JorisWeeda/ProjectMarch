// Copyright 2019 Project March.

#include <march_gazebo_plugins/walk_controller.h>
#include <gazebo/physics/physics.hh>

namespace gazebo
{
WalkController::WalkController(physics::ModelPtr model) : ObstacleController(model)
{
  this->swing_step_size_ = 0.7;  // This estimate will be adjusted every step

  this->p_pitch_ = 120;
  this->p_pitch_off_ = 20;
  this->d_pitch_ = 40000;
  this->d_pitch_off_ = 100;

  this->p_roll_ = 150;
  this->p_roll_off_ = 20;
  this->d_roll_ = 40000;
  this->d_roll_off_ = 100;

  this->p_yaw_ = 1000;
  this->p_yaw_off_ = 50;
  this->d_yaw_ = 40000;
  this->d_yaw_off_ = 100;
}

}  // namespace gazebo
