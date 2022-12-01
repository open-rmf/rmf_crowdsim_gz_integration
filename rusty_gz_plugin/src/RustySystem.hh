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
#ifndef RUSTY_GZ_SYSTEM_HH_
#define RUSTY_GZ_SYSTEM_HH_

#include <memory>
#include <gz/sim/System.hh>
#include "rust_interface.h"
#include <rclcpp/node.hpp>

#include <chart_sim_msgs/msg/agent_go_to_place.hpp>
#include <chart_sim_msgs/msg/event_finished.hpp>
#include <chart_sim_msgs/msg/agent_set_animation.hpp>

namespace rusty
{
  /// \brief A plugin that validates target identification reports.
  class RustySystem:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    /// \brief Constructor
    public: RustySystem();
    /// \brief Destructor
    public: ~RustySystem();

    // Documentation inherited
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
              const gz::sim::UpdateInfo &_info,
              const gz::sim::EntityComponentManager &_ecm) override;

    private: simulation_binding_t* crowdsim;
    public:
      std::unordered_map<gz::sim::Entity, uint64_t> robot_map;
      std::unordered_map<gz::sim::Entity, uint64_t> agent_map;
      std::unordered_map<std::string, uint64_t> agent_name_map;
      std::unordered_map<uint64_t, std::string> reverse_agent_name_map;
      std::unordered_map<std::string, uint64_t> spawning_queue;

      // (agent_id -> (goal_id -> request_id))
      std::unordered_map<uint64_t, std::unordered_map<uint64_t, uint64_t>> pending_goals;

      rclcpp::Node::SharedPtr node;

      using AgentGoToPlace = chart_sim_msgs::msg::AgentGoToPlace;
      rclcpp::Subscription<AgentGoToPlace>::SharedPtr agent_go_to_place_sub;

      using AgentSetAnimation = chart_sim_msgs::msg::AgentSetAnimation;
      rclcpp::Publisher<AgentSetAnimation>::SharedPtr agent_set_animation_pub;

      using EventFinished = chart_sim_msgs::msg::EventFinished;
      rclcpp::Publisher<EventFinished>::SharedPtr event_finished_pub;

  };
}

#endif
