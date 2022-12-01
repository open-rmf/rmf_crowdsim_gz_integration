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

extern "C" typedef struct SimulationBinding simulation_binding_t;

extern "C"{
/// \brief This stucture is a generic 2D Vector
struct Position
{
    /// \brief X position
    double x;
    /// \brief Y position
    double y;
    /// \brief The yaw for the agent
    double yaw;
    /// \brief Visbility: Used by crowdsim_query_position to return existence
    /// of an object. 0 if ok -1 if object is missing.
    int visible;
};
}

/// Callback type for listening to spawn events.
typedef void (*spawn_cb_t) (void* system, uint64_t id, const char* model, double x, double y, double yaw);

/// \brief Create a new crowdsim instance
/// \param[in] file_path - File path.
/// \param[in] cb - The spawn callback.
extern "C" simulation_binding_t * crowdsim_new(
    const char* agents_path,
    const char* nav_path,
    void* system,
    spawn_cb_t cb);

/// \brief Free the crowdsim instance
/// \param[in] t - Simulation instance to destroy.
extern "C" void crowdsim_free(simulation_binding_t *t);

/// \brief Query position at current simulation time step.
/// \param[in] t - Simulation instance
/// \param[in] id - The agent in question's id
/// \returns The agent position. If the agent cannot be found (likely due to
/// removal), then set the visiblity field to -1 else vibility field will be
/// 1.
extern "C" Position crowdsim_query_position(
    simulation_binding_t* t, uint64_t id);

/// \brief Runs the simulation for a given time step.
/// \param[in] t - Simulation instance
/// \param[in] timestep - amount of time in seconds to step the simulation.
extern "C" void crowdsim_run(
    simulation_binding_t* t,
    float timestep);

/// \brief If the model_name matches the name of a robot that needs to be
/// updated then get its ID. Otherwise get -1.
extern "C" int64_t crowdsim_get_robot_id(
    simulation_binding_t* t,
    const char* model_name);

/// Update the position of a robot
extern "C" void crowdsim_update_robot_position(
    simulation_binding_t* t,
    uint64_t robot_id,
    Position position);
