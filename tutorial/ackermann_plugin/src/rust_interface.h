#pragma once
/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/// This file contains C-bindings to rmf_crowdsim.

#include <cstdint>

extern "C" typedef struct SimulationBinding
{
  /// \brief Calculated speed of left wheel joint(s)
  double leftJointSpeed{0};

  /// \brief Calculated speed of right wheel joint(s)
  double rightJointSpeed{0};

  /// \brief Calculated speed of left joint
  double leftSteeringJointSpeed{0};

  /// \brief Calculated speed of right joint
  double rightSteeringJointSpeed{0};

  /// \brief Distance between left and right wheels
  double wheelSeparation{1.0};

  /// \brief Distance between left and right wheel kingpins
  double kingpinWidth{0.8};

  /// \brief Distance between front and back wheels
  double wheelBase{1.0};

  /// \brief Maximum turning angle to limit steering to
  double steeringLimit{0.5};

  /// \brief Wheel radius
  double wheelRadius{0.2};
} simulation_binding_t;

extern "C" simulation_binding_t * simulation_binding_new(
  // double leftJointSpeed,
  // double rightJointSpeed,
  // double ,
  // double rightSteeringJointSpeed,
  double wheelSeparation,
  double kingpinWidth,
  double wheelBase,
  double steeringLimit,
  double wheelRadius,
  double minVel,
  double maxVel,
  double minAccel,
  double maxAccel,
  double minJerk,
  double maxJerk,
  double odomPubPeriod
);
extern "C" void simulation_binding_free(simulation_binding_t *t);

extern "C" void update_velocity(simulation_binding_t *t,
    double& linear, double& angular,
    double leftSteeringPos, double rightSteeringPos,
    double f64);

extern "C" double get_left_joint_speed(simulation_binding_t *t);
extern "C" double get_right_joint_speed(simulation_binding_t *t);
extern "C" double get_left_steering_speed(simulation_binding_t *t);
extern "C" double get_right_steering_speed(simulation_binding_t *t);

extern "C" bool calculate_odometry(simulation_binding_t *t,
    double leftPos, double rightPos,
    double leftSteeringPos, double rightSteeringPos,
    double simTime);

extern "C" double get_odom_x(simulation_binding_t *t);
extern "C" double get_odom_y(simulation_binding_t *t);
extern "C" double get_odom_yaw(simulation_binding_t *t);
extern "C" double get_odom_linear(simulation_binding_t *t);
extern "C" double get_odom_angular(simulation_binding_t *t);
