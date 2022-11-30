#![feature(core_ffi_c)]

use rmf_crowdsim::local_planners::zanlungo::Zanlungo;
use rmf_crowdsim::source_sink::source_sink::{PoissonCrowd, SourceSink};
use rmf_crowdsim::spatial_index::location_hash_2d::LocationHash2D;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use rmf_crowdsim::rmf::RMFPlanner;
use rmf_crowdsim::*;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::ffi::{c_int, c_float, c_char, CStr};
use std::fs;
use serde::Deserialize;
use rmf_site_format::legacy::nav_graph::NavGraph;

const RESOURCE_ENVIRONMENT_VARIABLE: &str = "GZ_SIM_RESOURCE_PATH";

////////////////////////////////////////////////////////////////////////////////
/// Globals go here.
#[derive(Debug, Clone, Deserialize)]
pub struct CrowdSimConfig {
    level: String,
    #[serde(default)]
    agents: HashMap<String, PersistentConfig>,
    #[serde(default)]
    robots: Vec<String>,
    #[serde(default)]
    source_sinks: Vec<SourceSinkConfig>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct PersistentConfig {
    /// String name of starting location
    start: String,
    /// Name of model to use for this agent
    model: String,
    // TODO(MXG): Consider allowing the goal_radius to change for each request
    #[serde(default = "default_goal_radius")]
    goal_radius: f64,
    #[serde(default)]
    avoidance: AvoidanceConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SourceSinkConfig {
    source: String,
    model: String,
    waypoints: Vec<String>,
    rate: f64,
    #[serde(default = "default_source_range")]
    source_range: [f64; 2],
    #[serde(default = "default_goal_radius")]
    goal_radius: f64,
    #[serde(default)]
    avoidance: AvoidanceConfig,
}

#[derive(Debug, Clone, Deserialize)]
pub struct AvoidanceConfig {
    #[serde(default = "default_agent_radius")]
    agent_radius: f64,
    #[serde(default = "default_force_distance")]
    force_distance: f64,
    #[serde(default = "default_agent_mass")]
    mass: f64,
    /// This is passed to Zanglungo::agent_scale
    #[serde(default = "default_force_coefficient")]
    force_coefficient: f64,
    #[serde(default = "default_eyesight_range")]
    eyesight_range: f64,
}

impl From<&AvoidanceConfig> for Zanlungo {
    fn from(av: &AvoidanceConfig) -> Self {
        Zanlungo::new(
            av.force_coefficient,
            0.0, // unused
            0.0, // unused
            av.force_distance,
            av.mass,
            av.agent_radius,
        )
    }
}

impl Default for AvoidanceConfig {
    fn default() -> Self {
        AvoidanceConfig {
            agent_radius: default_agent_radius(),
            force_distance: default_force_distance(),
            mass: default_agent_mass(),
            force_coefficient: default_force_coefficient(),
            eyesight_range: default_eyesight_range(),
        }
    }
}

fn default_agent_radius() -> f64 {
    0.1
}

fn default_goal_radius() -> f64 {
    0.1
}

fn default_source_range() -> [f64; 2] {
    [0.0, 0.0]
}

fn default_force_distance() -> f64 {
    5.0
}

fn default_agent_mass() -> f64 {
    0.1
}

fn default_force_coefficient() -> f64 {
    1.0
}

fn default_eyesight_range() -> f64 {
    5.0
}

/// List of SourceSink components
static mut SIM_CONFIG: Vec<CrowdSimConfig> = vec!();

/// Spawn callback
pub struct SpawnCBIntegration {
    pub call_back: extern fn(u64, f64, f64) -> ()
}
static mut GZ_SPAWN_CB: Option<SpawnCBIntegration> = None;

struct SimulationModel {
    crowd_sim: Simulation<LocationHash2D>,
    //high_level_planner: Arc<StubHighLevelPlan>,
    //local_planner: Arc<NoLocalPlan>,
}
static mut SIM_MODEL: Option<Arc<Mutex<SimulationModel>>> = None;

static mut FILE_PATH: &str = "";

#[repr(C)]
pub struct Position
{
    x: c_float,
    y: c_float,
    visible: c_int
}

#[repr(C)]
pub struct SimulationBinding
{
    crowd_sim: Simulation<LocationHash2D>,
    agent_map: HashMap<String, usize>,
    robot_map: HashMap<String, usize>,
    spawn_callback: SpawnCBIntegration,
}

#[no_mangle]
pub extern "C" fn crowdsim_new(
    agent_path: *const c_char,
    nav_path: *const c_char,
    spawn_cb: extern fn (u64, f64, f64) -> ()
) -> *mut SimulationBinding
{
    // TODO(arjo): Calculate size based on rmf_planner
    let stub_spatial = spatial_index::location_hash_2d::LocationHash2D::new(
        1000f64,
        1000f64,
        20f64,
        Point::new(-500f64, -500f64),
    );

    let mut crowd_sim = Simulation::new(stub_spatial);

    let root = match std::env::var(RESOURCE_ENVIRONMENT_VARIABLE) {
        Ok(s) => s,
        Err(err) => {
            println!("Could not get {RESOURCE_ENVIRONMENT_VARIABLE}: {err:?}");
            return std::ptr::null_mut();
        }
    };

    let nav_filename = match unsafe { CStr::from_ptr(nav_path) }.to_str() {
        Ok(s) => s,
        Err(err) => {
            println!("Could not interpret nav graph file name {nav_path:?}");
            return std::ptr::null_mut();
        }
    };
    let nav_f = match std::fs::File::open(nav_filename) {
        Ok(f) => f,
        Err(err) => {
            println!("Could not open {nav_filename}");
            return std::ptr::null_mut();
        }
    };
    let nav: NavGraph = match serde_yaml::from_reader(nav_f) {
        Ok(r) => r,
        Err(err) => {
            println!("Failed to parse nav graph {nav_filename}: {err:?}");
            return std::ptr::null_mut();
        }
    };

    let agent_filename = match unsafe { CStr::from_ptr(agent_path) }.to_str() {
        Ok(s) => s,
        Err(err) => {
            println!("Could not interpret agent file name {agent_path:?}");
            return std::ptr::null_mut();
        }
    };
    let agent_f = match std::fs::File::open(agent_filename) {
        Ok(f) => f,
        Err(err) => {
            println!("Could not open {agent_filename}");
            return std::ptr::null_mut();
        }
    };
    let sim_config: CrowdSimConfig = match serde_yaml::from_reader(agent_f) {
        Ok(r) => r,
        Err(err) => {
            println!("Failed to parse {agent_filename}: {err:?}");
            return std::ptr::null_mut();
        }
    };

    let nav_level = match nav.levels.get(&sim_config.level) {
        Some(l) => l,
        None => {
            println!(
                "Nav graph [{nav_filename}] is missing requested level [{}]",
                sim_config.level,
            );
            return std::ptr::null_mut();
        }
    };

    let occupancy = match &nav_level.occupancy {
        Some(o) => o,
        None => {
            println!(
                "Nav graph [{nav_filename}] is missing occupancy for requested level [{}]",
                sim_config.level,
            );
            return std::ptr::null_mut();
        }
    };

    let locations: HashMap<String, Vec2f> = nav_level.vertices.iter()
        .filter_map(|v| {
            if v.2.name.is_empty() {
                None
            } else {
                Some((v.2.name.clone(), Vec2f::new(v.0 as f64, v.1 as f64)))
            }
        })
        .collect();

    for ss in &sim_config.source_sinks {
        let high_level_planner = Arc::new(Mutex::new(
            RMFPlanner::from_occupancy(&occupancy, ss.avoidance.agent_radius)
        ));
        let local_planner = Arc::new(Mutex::new(Zanlungo::from(&ss.avoidance)));
        let source = match locations.get(&ss.source) {
            Some(s) => *s,
            None => {
                println!(
                    "Could not find a vertex for source [{}] in the nav graph {nav_filename}",
                    ss.source,
                );
                return std::ptr::null_mut();
            }
        };

        let mut waypoints = Vec::new();
        for wp in &ss.waypoints {
            let wp = match locations.get(wp) {
                Some(wp) => wp,
                None => {
                    println!("Could not fine a vertex for waypoint [{}] in the nav graph {nav_filename}", wp);
                    return std::ptr::null_mut();
                }
            };
            waypoints.push(*wp);
        }

        crowd_sim.add_source_sink(Arc::new(SourceSink {
            source,
            waypoints,
            radius_sink: ss.goal_radius,
            crowd_generator: Arc::new(PoissonCrowd::new(ss.rate)),
            high_level_planner,
            local_planner,
            agent_eyesight_range: ss.avoidance.eyesight_range,
            source_range: ss.source_range.into(),
            loop_forever: false,
        }));
    }

    let mut agent_map: HashMap<String, usize> = HashMap::new();
    for (name, agent) in &sim_config.agents {
        let start = match locations.get(&agent.start) {
            Some(s) => *s,
            None => {
                println!("Could not find a vertex for agent [{name}] start [{}]", agent.start);
                return std::ptr::null_mut();
            }
        };
        let high_level_planner = Arc::new(Mutex::new(
            RMFPlanner::from_occupancy(&occupancy, agent.avoidance.agent_radius)
        ));
        let local_planner = Arc::new(Mutex::new(Zanlungo::from(&agent.avoidance)));

        let id = match crowd_sim.add_persistent_agent(
            start, agent.goal_radius, high_level_planner, local_planner, agent.avoidance.eyesight_range
        ) {
            Ok(id) => id,
            Err(err) => {
                println!("Failed to add persistent agent: {err:?}");
                return std::ptr::null_mut();
            }
        };
        agent_map.insert(name.clone(), id);
    }

    let mut robot_map: HashMap<String, usize> = HashMap::new();
    for name in &sim_config.robots {
        let id = match crowd_sim.add_obstacle(Vec2f::zeros()) {
            Ok(id) => id,
            Err(err) => {
                println!("Failed to add obstacle: {err:?}");
                return std::ptr::null_mut();
            }
        };
        robot_map.insert(name.clone(), id);
    }

    let event_listener = Arc::new(Mutex::new(
        CrowdEventListener::new(SpawnCBIntegration {
            call_back: spawn_cb})));
    crowd_sim.add_event_listener(event_listener.clone());

    Box::into_raw(Box::new(SimulationBinding
    {
        crowd_sim,
        agent_map,
        robot_map,
        spawn_callback: SpawnCBIntegration { call_back: spawn_cb },
    }))
}

#[no_mangle]
pub extern "C" fn crowdsim_free(ptr: *mut SimulationBinding)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        Box::from_raw(ptr);
    }
}


#[no_mangle]
pub extern "C" fn crowdsim_query_position(
    ptr: *mut SimulationBinding,
    agent_id: u64) -> Position
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let agent = sim_binding.crowd_sim.agents.get(&(agent_id as usize));
    if let Some(agent) = agent {
        return Position{
            x: agent.position.x as f32,
            y: agent.position.y as f32,
            visible: 1
        };
    }
    return Position{x: 0.0, y: 0.0, visible: -1};
}


#[no_mangle]
pub extern "C" fn crowdsim_run(
    ptr: *mut SimulationBinding,
    dt: f32)
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    if dt <= 0.0 {
        println!("Got negative dt");
        return;
    }
    let step_size = std::time::Duration::new(
        dt.floor() as u64, ((dt - dt.floor())*1e9) as u32);
    sim_binding.crowd_sim.step(step_size);
}
////////////////////////////////////////////////////////////////////////////////
struct CrowdEventListener {
    correspondence: HashMap<u64, u64>,
    spawn_callback: SpawnCBIntegration
}

impl CrowdEventListener {
    pub fn new(spawn_callback: SpawnCBIntegration) -> Self {
        Self{
            correspondence: HashMap::new(),
            spawn_callback
        }
    }
}

impl EventListener for CrowdEventListener {
    fn agent_spawned(&mut self, position: Vec2f, agent: AgentId) {
        (self.spawn_callback.call_back)(agent as u64, position.x, position.y);
    }

    /// Called each time an agent is destroyed
    fn agent_destroyed(&mut self, agent: AgentId) {
        println!("Removed {}", agent);
    }
}
