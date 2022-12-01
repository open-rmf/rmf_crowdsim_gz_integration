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
      std::unordered_map<uint64_t, std::string> reverse_agent_map;
      std::unordered_map<std::string, uint64_t> spawning_queue;
  };
}

#endif
