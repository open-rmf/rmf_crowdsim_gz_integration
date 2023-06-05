/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "AckermannSteering.hh"

#include <gz/msgs/odometry.pb.h>
#include "rust_interface.h"

#include <climits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Angle.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;

namespace rust_ackerman
{
class AckermannSteeringPrivate
{
  /// \brief Callback for velocity subscription
  /// \param[in] _msg Velocity message
  public: void OnCmdVel(const gz::msgs::Twist &_msg);

  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance./// \brief Velocity command.
  public: void UpdateOdometry(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Entity of the left joint
  public: std::vector<Entity> leftJoints;

  /// \brief Entity of the right joint
  public: std::vector<Entity> rightJoints;

  /// \brief Entity of the left steering joint
  public: std::vector<Entity> leftSteeringJoints;

  /// \brief Entity of the right steering joint
  public: std::vector<Entity> rightSteeringJoints;

  /// \brief Name of left joint
  public: std::vector<std::string> leftJointNames;

  /// \brief Name of right joint
  public: std::vector<std::string> rightJointNames;

  /// \brief Name of left steering joint
  public: std::vector<std::string> leftSteeringJointNames;

  /// \brief Name of right steering joint
  public: std::vector<std::string> rightSteeringJointNames;

  public: simulation_binding_t * simulationBindings{nullptr};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Ackermann steering odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Ackermann tf message publisher.
  public: transport::Node::Publisher tfPub;

  /// \brief Last target velocity requested.
  public: msgs::Twist targetVel;

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;
};

//////////////////////////////////////////////////
AckermannSteering::AckermannSteering()
  : dataPtr(std::make_unique<AckermannSteeringPrivate>())
{
}

//////////////////////////////////////////////////
void AckermannSteering::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "AckermannSteering plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto ptr = const_cast<sdf::Element *>(_sdf.get());

  // Get params from SDF
  sdf::ElementPtr sdfElem = ptr->GetElement("left_joint");
  while (sdfElem)
  {
    this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("left_joint");
  }
  sdfElem = ptr->GetElement("right_joint");
  while (sdfElem)
  {
    this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("right_joint");
  }
  sdfElem = ptr->GetElement("left_steering_joint");
  while (sdfElem)
  {
    this->dataPtr->leftSteeringJointNames.push_back(
                          sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("left_steering_joint");
  }
  sdfElem = ptr->GetElement("right_steering_joint");
  while (sdfElem)
  {
    this->dataPtr->rightSteeringJointNames.push_back(
                           sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("right_steering_joint");
  }

  double wheelSeparation = _sdf->Get<double>("wheel_separation", 1.0).first;
  double kingpinWidth = _sdf->Get<double>("kingpin_width", 0.8).first;
  double wheelBase = _sdf->Get<double>("wheel_base", 1.0).first;
  double steeringLimit = _sdf->Get<double>("steering_limit", 0.5).first;
  double wheelRadius = _sdf->Get<double>("wheel_radius", 0.2).first;

  double minVel = std::numeric_limits<double>::min();
  double maxVel = std::numeric_limits<double>::max();
  double minAccel = std::numeric_limits<double>::min();
  double maxAccel = std::numeric_limits<double>::max();
  double minJerk = std::numeric_limits<double>::min();
  double maxJerk = std::numeric_limits<double>::max();
  // Parse speed limiter parameters.
  if (_sdf->HasElement("min_velocity"))
  {
    minVel = _sdf->Get<double>("min_velocity");
  }
  if (_sdf->HasElement("max_velocity"))
  {
    maxVel = _sdf->Get<double>("max_velocity");
  }
  if (_sdf->HasElement("min_acceleration"))
  {
    minAccel = _sdf->Get<double>("min_acceleration");
  }
  if (_sdf->HasElement("max_acceleration"))
  {
    maxAccel = _sdf->Get<double>("max_acceleration");
  }
  if (_sdf->HasElement("min_jerk"))
  {
    minJerk = _sdf->Get<double>("min_jerk");
  }
  if (_sdf->HasElement("max_jerk"))
  {
    maxJerk = _sdf->Get<double>("max_jerk");
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  std::chrono::steady_clock::duration odomPubPeriod{0};
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  this->dataPtr->simulationBindings = simulation_binding_new(
    wheelSeparation,
    kingpinWidth,
    wheelBase,
    steeringLimit,
    wheelRadius,
    minVel,
    maxVel,
    minAccel,
    maxAccel,
    minJerk,
    maxJerk,
    std::chrono::duration_cast<std::chrono::seconds>(odomPubPeriod).count());

  // Subscribe to commands
  std::vector<std::string> topics;
  if (_sdf->HasElement("topic"))
  {
    topics.push_back(_sdf->Get<std::string>("topic"));
  }
  topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
  auto topic = validTopic(topics);
  if (topic.empty())
  {
    gzerr << "AckermannSteering plugin received invalid model name "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->node.Subscribe(topic, &AckermannSteeringPrivate::OnCmdVel,
      this->dataPtr.get());

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/odometry");
  auto odomTopic = validTopic(odomTopics);
  if (topic.empty())
  {
    gzerr << "AckermannSteering plugin received invalid model name "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  std::vector<std::string> tfTopics;
  if (_sdf->HasElement("tf_topic"))
  {
    tfTopics.push_back(_sdf->Get<std::string>("tf_topic"));
  }
  tfTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
    "/tf");
  auto tfTopic = validTopic(tfTopics);
  if (tfTopic.empty())
  {
    gzerr << "AckermannSteering plugin invalid tf topic name "
           << "Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  gzmsg << "AckermannSteering subscribing to twist messages on [" <<
      topic << "]" << std::endl;
}

//////////////////////////////////////////////////
void AckermannSteering::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (this->dataPtr->leftJoints.empty() ||
      this->dataPtr->rightJoints.empty() ||
      this->dataPtr->leftSteeringJoints.empty() ||
      this->dataPtr->rightSteeringJoints.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->leftJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->rightJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find right joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }
    for (const std::string &name : this->dataPtr->leftSteeringJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftSteeringJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left steering joint ["
                << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->rightSteeringJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightSteeringJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find right steering joint [" <<
            name << "] for model [" << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty() ||
      this->dataPtr->leftSteeringJoints.empty() ||
      this->dataPtr->rightSteeringJoints.empty())
    return;

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    gzmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  for (Entity joint : this->dataPtr->leftJoints)
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);

    auto leftJointSpeed = get_left_joint_speed(this->dataPtr->simulationBindings);
    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          joint, components::JointVelocityCmd({leftJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({leftJointSpeed});
    }
  }

  for (Entity joint : this->dataPtr->rightJoints)
  {
    // Update wheel velocity
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    auto rightJointSpeed = get_right_joint_speed(this->dataPtr->simulationBindings);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(joint,
          components::JointVelocityCmd({rightJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({rightJointSpeed});
    }
  }

  // Update steering
  for (Entity joint : this->dataPtr->leftSteeringJoints)
  {
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    auto leftJointSpeed = get_left_steering_speed(this->dataPtr->simulationBindings);
    if (vel == nullptr)
    {
      _ecm.CreateComponent(
          joint, components::JointVelocityCmd({leftJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({leftJointSpeed});
    }
  }

  for (Entity joint : this->dataPtr->rightSteeringJoints)
  {
    auto vel = _ecm.Component<components::JointVelocityCmd>(joint);
    auto rightJointSpeed = get_right_steering_speed(this->dataPtr->simulationBindings);

    if (vel == nullptr)
    {
      _ecm.CreateComponent(joint,
          components::JointVelocityCmd({rightJointSpeed}));
    }
    else
    {
      *vel = components::JointVelocityCmd({rightJointSpeed});
    }
  }

  // Create the left and right side joint position components if they
  // don't exist.
  auto leftPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->leftJoints[0]);
  if (!leftPos)
  {
    _ecm.CreateComponent(this->dataPtr->leftJoints[0],
        components::JointPosition());
  }

  auto rightPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->rightJoints[0]);
  if (!rightPos)
  {
    _ecm.CreateComponent(this->dataPtr->rightJoints[0],
        components::JointPosition());
  }

  auto leftSteeringPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->leftSteeringJoints[0]);
  if (!leftSteeringPos)
  {
    _ecm.CreateComponent(this->dataPtr->leftSteeringJoints[0],
        components::JointPosition());
  }

  auto rightSteeringPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->rightSteeringJoints[0]);
  if (!rightSteeringPos)
  {
    _ecm.CreateComponent(this->dataPtr->rightSteeringJoints[0],
        components::JointPosition());
  }
}

//////////////////////////////////////////////////
void AckermannSteering::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->UpdateVelocity(_info, _ecm);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateOdometry(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::UpdateOdometry");
  // Initialize, if not already initialized.

  if (this->leftJoints.empty() || this->rightJoints.empty() ||
      this->leftSteeringJoints.empty() || this->rightSteeringJoints.empty())
    return;

  // Get the first joint positions for the left and right side.
  auto leftPos = _ecm.Component<components::JointPosition>(this->leftJoints[0]);
  auto rightPos = _ecm.Component<components::JointPosition>(
      this->rightJoints[0]);
  auto leftSteeringPos = _ecm.Component<components::JointPosition>(
      this->leftSteeringJoints[0]);
  auto rightSteeringPos = _ecm.Component<components::JointPosition>(
      this->rightSteeringJoints[0]);

  // Abort if the joints were not found or just created.
  if (!leftPos || !rightPos || leftPos->Data().empty() ||
      rightPos->Data().empty() ||
      !leftSteeringPos || !rightSteeringPos ||
      leftSteeringPos->Data().empty() ||
      rightSteeringPos->Data().empty())
  {
    return;
  }

  if(!calculate_odometry(this->simulationBindings,
                     leftPos->Data()[0], rightPos->Data()[0],
                     leftSteeringPos->Data()[0], rightSteeringPos->Data()[0],
                     std::chrono::duration<double>(_info.simTime).count()))
  {
    return;
  }

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(get_odom_x(this->simulationBindings));
  msg.mutable_pose()->mutable_position()->set_y(get_odom_y(this->simulationBindings));

  math::Quaterniond orientation(0, 0, get_odom_yaw(this->simulationBindings));
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(get_odom_linear(this->simulationBindings));
  msg.mutable_twist()->mutable_angular()->set_z(get_odom_angular(this->simulationBindings));

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tfMsg;
  auto *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the message
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::UpdateVelocity(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("AckermannSteering::UpdateVelocity");

  double linVel;
  double angVel;
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    linVel = this->targetVel.linear().x();
    angVel = this->targetVel.angular().z();
  }

  auto leftSteeringPos = _ecm.Component<components::JointPosition>(
      this->leftSteeringJoints[0]);
  auto rightSteeringPos = _ecm.Component<components::JointPosition>(
      this->rightSteeringJoints[0]);
  // Abort if the joints were not found or just created.
  if (!leftSteeringPos || !rightSteeringPos ||
      leftSteeringPos->Data().empty() ||
      rightSteeringPos->Data().empty())
  {
    return;
  }

  update_velocity(this->simulationBindings, linVel, angVel,
                  leftSteeringPos->Data()[0], rightSteeringPos->Data()[0],
                  std::chrono::duration<double>(_info.dt).count());
}

//////////////////////////////////////////////////
void AckermannSteeringPrivate::OnCmdVel(const msgs::Twist &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->targetVel = _msg;
}
}

GZ_ADD_PLUGIN(rust_ackerman::AckermannSteering,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate,
              gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(rust_ackerman::AckermannSteering,
                    "rust_ackerman::AckermannSteering")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(rust_ackerman::AckermannSteering,
                    "rust_ackerman::AckermannSteering")
