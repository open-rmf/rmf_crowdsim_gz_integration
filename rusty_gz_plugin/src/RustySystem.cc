#include "rust_interface.h"

#include "RustySystem.hh"
#include <gz/plugin/Register.hh>

#include <gz/msgs/entity_factory.pb.h>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Static.hh>

#include <gz/transport/Node.hh>

#include <gz/common/StringUtils.hh>

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
    void* v_system, uint64_t id, const char* model, double x, double y, double yaw)
{
    auto sdf = R"(
    <?xml version="1.0" ?>
    <sdf version='1.7'>
        <include>
            <uri>
            https://fuel.gazebosim.org/1.0/OpenRobotics/models/)"
            + std::string(model) + R"(/1
            </uri>
            <pose>)" + std::to_string(x) + " " + std::to_string(y) + " " +
            R"(0.0 0 0 )" + std::to_string(yaw) + R"(</pose>
        </include>
    </sdf>)";
    auto name = "generated_agent_" + std::to_string(id);
    createEntityFromStr(name, sdf);

    auto* system = static_cast<RustySystem*>(v_system);
    system->spawning_queue.insert({name, id});
}

extern "C" void moving_agent(void* v_system, uint64_t id)
{
  auto* system = static_cast<RustySystem*>(v_system);

}

extern "C" void idle_agent(void* v_system, uint64_t id)
{

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
  std::string agents;
  std::string nav;
  if (_sdf->HasElement("agents"))
  {
    agents = _sdf->Get<std::string>("agents");
  }
  else
  {
    gzerr << "Please specify a path using the <agents> tag!\n";
    return;
  }
  if (_sdf->HasElement("nav"))
  {
    nav = _sdf->Get<std::string>("nav");
  }
  else
  {
    gzerr << "Please specify a path using the <nav> tag!\n";
  }

  // Creates a new crowdsim instance
  this->crowdsim = crowdsim_new(
    agents.c_str(),
    nav.c_str(),
    (void*)(this),
    spawn_agent,
    moving_agent,
    idle_agent
  );
  
  worldName = _ecm.Component<components::Name>(_entity)->Data();
}

void RustySystem::PreUpdate(const gz::sim::UpdateInfo &_info,
                            gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
  {
    return;
  }

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
  _ecm.Each<components::Actor>(
    [&](const Entity &_entity, const components::Actor *)->bool
    {
      const auto it = this->agent_map.find(_entity);
      if (it == this->agent_map.end())
        return true;

      const auto agent_id = it->second;
      auto position = crowdsim_query_position(this->crowdsim, agent_id);
      if (position.visible < 0)
      {
        _ecm.RequestRemoveEntity(_entity, true);
        return true;
      }

      _ecm.Component<components::Pose>(_entity)->Data() = gz::math::Pose3d(
            position.x, position.y, 0.0, 0, 0, position.yaw);
      _ecm.SetChanged(_entity, components::Pose::typeId,
                      ComponentState::OneTimeChange);
      return true;
    });
}

void RustySystem::PostUpdate(
  const gz::sim::UpdateInfo &,
  const gz::sim::EntityComponentManager &_ecm)
{
  _ecm.EachNew<components::Model, components::Name>(
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

  std::vector<std::string> spawned;
  _ecm.EachNew<components::Actor, components::Name>(
    [&](const Entity &_entity, const components::Actor*, const components::Name* name) -> bool
    {
      const auto it = this->spawning_queue.find(name->Data());
      if (it == this->spawning_queue.end())
        return true;

      spawned.push_back(it->first);
      this->agent_map.insert({_entity, it->second});
      this->reverse_agent_map.insert({it->second, name->Data()});
      return true;
    });

  for (const auto& s : spawned)
  {
    this->spawning_queue.erase(s);
  }
}

GZ_ADD_PLUGIN(rusty::RustySystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
