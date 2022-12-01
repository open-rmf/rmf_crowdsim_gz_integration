#include "rust_interface.h"

#include "RustySystem.hh"
#include <gz/plugin/Register.hh>

#include <gz/msgs/entity_factory.pb.h>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>
#include <gz/sim/Util.hh>

#include <gz/transport/Node.hh>

#include <gz/common/StringUtils.hh>
#include <chart_sim_msgs/msg/agent_go_to_place.hpp>

#include <rclcpp/rclcpp.hpp>

using namespace rusty;

using namespace gz;
using namespace gz::sim;

std::string worldName;

gz::transport::Node node;

void createEntityFromStr(const std::string& name, const std::string& modelStr)
{
//! [call service create sphere]
  bool result;
  gz::msgs::EntityFactory req;
  gz::msgs::Boolean res;
  req.set_sdf(modelStr);
  req.set_name(name);

  bool executed = node.Request("/world/"+ worldName +"/create",
            req, 10000, res, result);
  if (executed)
  {
    if (result)
      gzdbg << "Entity was created : [" << res.data() << "]" << std::endl;
    else
    {
      gzerr << "Service call failed" << std::endl;
      return;
    }
  }
  else
    gzerr << "Service call timed out" << std::endl;
//! [call service create sphere]
}


extern "C" void spawn_agent(
    void* v_system, uint64_t id, const char* c_name, const char* model, double x, double y, double yaw)
{
    auto sdf = R"(
    <?xml version="1.0" ?>
    <sdf version='1.7'>
        <include>
            <uri>
            model://)" + std::string(model) +
            R"(
            </uri>
            <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " +
            R"(0.0 0 0 )" + std::to_string(yaw) + R"(</pose>
        </include>
    </sdf>)";
    auto name = std::string(c_name);
    createEntityFromStr(name, sdf);

    auto* system = static_cast<RustySystem*>(v_system);
    system->agent_name_map.insert({name, id});
    system->reverse_agent_name_map.insert({id, name});
}

extern "C" void moving_agent(void* v_system, uint64_t id)
{
  auto* system = static_cast<RustySystem*>(v_system);
  const auto it = system->reverse_agent_name_map.find(id);
  if (it == system->reverse_agent_name_map.end())
    return;

  system->agent_set_animation_pub->publish(
        chart_sim_msgs::build<chart_sim_msgs::msg::AgentSetAnimation>()
          .agent(it->second)
          .animation("walk")
          .cmd_id(-1)
        );
}

extern "C" void idle_agent(void* v_system, uint64_t id)
{
  auto* system = static_cast<RustySystem*>(v_system);
  const auto it = system->reverse_agent_name_map.find(id);
  if (it == system->reverse_agent_name_map.end())
    return;

  system->agent_set_animation_pub->publish(
        chart_sim_msgs::build<chart_sim_msgs::msg::AgentSetAnimation>()
          .agent(it->second)
          .animation("idle")
          .cmd_id(-1)
        );
}

extern "C" void goal_reached(void* v_system, uint64_t agent_id, uint64_t goal_id)
{
  auto* system = static_cast<RustySystem*>(v_system);
  const auto a_it = system->pending_goals.find(agent_id);
  if (a_it == system->pending_goals.end())
  {
    gzerr << "Could not find pending goals for agent [" << agent_id << "]\n";
    return;
  }

  const auto g_it = a_it->second.find(goal_id);
  if (g_it == a_it->second.end())
  {
    gzerr << "Could not find a command_id for goal [" << goal_id << "] of agent ["
          << agent_id << "]\n";
    return;
  }

  const auto cmd_id = g_it->second;
  system->event_finished_pub->publish(
        chart_sim_msgs::build<chart_sim_msgs::msg::EventFinished>()
        .cmd_id(cmd_id));

  // We no longer need to remember this goal.
  system->pending_goals[agent_id].erase(goal_id);
}

RustySystem::RustySystem()
{

}

RustySystem::~RustySystem()
{
  // TODO(arjo): Add a check if the crowdsim is actually inited
  crowdsim_free(this->crowdsim);
}

void RustySystem::Configure(const gz::sim::Entity &_entity,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &)
{
  if (!rclcpp::ok())
    rclcpp::init(0, nullptr);
  this->node = std::make_shared<rclcpp::Node>("crowdsim_plugin");
  const auto transient_qos = rclcpp::SystemDefaultsQoS()
        .keep_last(1)
        .reliable()
        .transient_local();

  this->agent_set_animation_pub = this->node->create_publisher<AgentSetAnimation>(
        "agent_set_animation", transient_qos);

  this->event_finished_pub = this->node->create_publisher<EventFinished>(
        "event_finished", transient_qos);

  this->agent_go_to_place_sub = this->node->create_subscription<AgentGoToPlace>(
      "agent_goto", transient_qos, [&](const AgentGoToPlace& msg)
    {
      const auto it = this->agent_name_map.find(msg.agent);
      if (it == this->agent_name_map.end())
      {
        gzerr << "Cannot find agent named [" << msg.agent << "]\n";
        return;
      }

      const auto agent_id = it->second;
      const auto goal_id = crowdsim_request_goal(
            this->crowdsim, agent_id, msg.place.c_str());
      if (goal_id < 0)
        return;

      this->pending_goals[agent_id][goal_id] = msg.cmd_id;
    });

  if (_sdf->HasElement("agents"))
  {
    this->agents = _sdf->Get<std::string>("agents");
  }
  else
  {
    gzerr << "Please specify a path using the <agents> tag!\n";
    return;
  }
  if (_sdf->HasElement("nav"))
  {
    this->nav = _sdf->Get<std::string>("nav");
  }
  else
  {
    gzerr << "Please specify a path using the <nav> tag!\n";
  }

  worldName = _ecm.Component<components::Name>(_entity)->Data();
}

void RustySystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }
  if (!this->initialized)
  {
    // Creates a new crowdsim instance
    this->crowdsim = crowdsim_new(
      this->agents.c_str(),
      this->nav.c_str(),
      (void*)(this),
      spawn_agent,
      moving_agent,
      idle_agent,
      goal_reached
    );

    this->InitializeRobotMap(_ecm);

    this->initialized = true;
  }
  rclcpp::spin_some(this->node);

  for (auto [e, robot_id] : this->robot_map)
  {
    if (const auto pose = _ecm.Component<components::Pose>(e))
    {
      const auto p = pose->Data().Pos();
      const auto euler = pose->Data().Rot().Euler();
      crowdsim_update_robot_position(
            this->crowdsim, robot_id,
            Position{(float)p.X(), (float)p.Y(), (float)euler[2], 1});
    }
    else
    {
      gzerr << "Failed to get pose for robot " << robot_id << "\n";
    }
  }

  crowdsim_run(
    this->crowdsim, std::chrono::duration<double>(_info.dt).count());
  _ecm.Each<components::Actor, components::Name>(
    [&](const Entity &_entity, const components::Actor *, components::Name *name)->bool
    {
      const auto it = this->agent_name_map.find(name->Data());
      if (it == this->agent_name_map.end())
        return true;

      const auto agent_id = it->second;
      auto position = crowdsim_query_position(this->crowdsim, agent_id);
      if (position.visible < 0)
      {
        _ecm.RequestRemoveEntity(_entity, true);
        // Bookkeeping to avoid maps growing indefinitely
        this->agent_name_map.erase(it);
        this->reverse_agent_name_map.erase(agent_id);
        return true;
      }

      if (!_ecm.EntityHasComponentType(_entity, components::AnimationName().TypeId()))
      {
        // Just created, add components and set them to defaults
        enableComponent<components::AnimationTime>(_ecm, _entity);
        enableComponent<components::AnimationName>(_ecm, _entity);
        enableComponent<components::TrajectoryPose>(_ecm, _entity);
        _ecm.Component<components::AnimationName>(_entity)->Data() = "walk";
        _ecm.SetChanged(_entity, components::AnimationName::typeId,
                        ComponentState::OneTimeChange);
      }

      // Make sure we have all the components we need
      _ecm.Component<components::TrajectoryPose>(_entity)->Data() = gz::math::Pose3d(
            position.x, position.y, 0.0, 0, 0, position.yaw);
      _ecm.Component<components::AnimationTime>(_entity)->Data() += _info.dt;
      _ecm.SetChanged(_entity, components::TrajectoryPose::typeId,
                      ComponentState::PeriodicChange);
      _ecm.SetChanged(_entity, components::AnimationTime::typeId,
                      ComponentState::PeriodicChange);
      return true;
    });
}

void RustySystem::InitializeRobotMap(
  const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.Each<components::Model, components::Name>(
    [&](const Entity &_entity, const components::Model*, const components::Name* name) -> bool
    {
      const int64_t robot_id = crowdsim_get_robot_id(
            this->crowdsim, name->Data().c_str());
      if (robot_id < 0)
      {
        return true;
      }

      this->robot_map.insert({_entity, robot_id});
      return true;
    });
}

GZ_ADD_PLUGIN(rusty::RustySystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
