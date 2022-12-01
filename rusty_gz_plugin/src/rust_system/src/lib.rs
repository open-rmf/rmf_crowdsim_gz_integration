#![feature(core_ffi_c)]

use rmf_crowdsim::local_planners::zanlungo::Zanlungo;
use rmf_crowdsim::source_sink::source_sink::{PoissonCrowd, SourceSink};
use rmf_crowdsim::spatial_index::location_hash_2d::LocationHash2D;
use rmf_crowdsim::spatial_index::spatial_index::SpatialIndex;
use rmf_crowdsim::rmf::RMFPlanner;
use rmf_crowdsim::*;
use std::sync::{Arc, Mutex};
use std::collections::HashMap;
use std::ffi::{c_int, c_double, c_char, CStr, c_void, CString};
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
    #[serde(default = "default_orientation")]
    orientation: f64,
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
    #[serde(default = "default_orientation")]
    orientation: f64,
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

fn default_orientation() -> f64 {
    0.0
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

type SpawnFn = extern fn(*mut c_void, u64, *const c_char, *const c_char, f64, f64, f64) -> ();
type MovingFn = extern fn(*mut c_void, u64);
type IdleFn = extern fn(*mut c_void, u64);
type GoalReachedFn = extern fn(*mut c_void, u64, u64);

/// Spawn callback
pub struct Callbacks {
    pub system: *mut c_void,
    pub spawn: SpawnFn,
    pub moving: MovingFn,
    pub idle: IdleFn,
    pub reached: GoalReachedFn,
}

impl EventListener for Callbacks {
    fn agent_spawned(&mut self, position: Vec2f, yaw: f64, name: String, model: &String, agent: AgentId) {
        unsafe {
            let name_c_str = match CString::new(name) {
                Ok(name) => name,
                Err(err) => {
                    println!("Unable to convert name while triggering agent_spawned: {err:?}");
                    return;
                }
            };
            let model_c_str = match CString::new(model.clone()) {
                Ok(model) => model,
                Err(err) => {
                    println!("Unable to convert model while triggering agent_spawned: {err:?}");
                    return;
                }
            };
            (self.spawn)(self.system, agent as u64, name_c_str.as_ptr(), model_c_str.as_ptr(), position.x, position.y, yaw);
        }
    }

    fn goal_reached(&mut self, goal_id: usize, agent: AgentId) {
        (self.reached)(self.system, agent as u64, goal_id as u64);
    }

    fn agent_moving(&mut self, agent: AgentId) {
        (self.moving)(self.system, agent as u64);
    }

    fn agent_idle(&mut self, agent: AgentId) {
        (self.idle)(self.system, agent as u64);
    }

    /// Called each time an agent is destroyed
    fn agent_destroyed(&mut self, agent: AgentId) {
        println!("Removed {}", agent);
    }
}

#[repr(C)]
pub struct Position {
    x: c_double,
    y: c_double,
    yaw: c_double,
    visible: c_int,
}

impl From<Position> for Vec2f {
    fn from(value: Position) -> Self {
        Vec2f::new(value.x as f64, value.y as f64)
    }
}

#[repr(C)]
pub struct SimulationBinding
{
    crowd_sim: Simulation<LocationHash2D>,
    agent_map: HashMap<String, usize>,
    robot_map: HashMap<String, Option<usize>>,
    location_map: HashMap<String, Vec2f>,
}

// Utility function to iterate over paths in the environment variable and find a valid file
// TODO should really use PathBuf for this
fn open_file(
    filename: &str,
) -> Option<std::fs::File>
{
    let roots: Vec<String> = match std::env::var(RESOURCE_ENVIRONMENT_VARIABLE) {
        Ok(env_var) => {
            let mut res = Vec::<String>::new();
            for path in env_var.split(":") {
                res.push(String::from(path));
            }
            res
        }
        Err(err) => {
            println!("Could not get {RESOURCE_ENVIRONMENT_VARIABLE}: {err:?}");
            return None;
        }
    };

    for root in roots {
        let concatenated = String::from(root) + filename;
        match std::fs::File::open(concatenated) {
            Ok(f) => {
                return Some(f);
            }
            Err(_) => {
                continue;
            }
        };
    }
    None
}

#[no_mangle]
pub extern "C" fn crowdsim_new(
    agent_path: *const c_char,
    nav_path: *const c_char,
    system: *mut c_void,
    spawn: SpawnFn,
    moving: MovingFn,
    idle: IdleFn,
    reached: GoalReachedFn,
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

    let nav_filename = match unsafe { CStr::from_ptr(nav_path) }.to_str() {
        Ok(s) => s,
        Err(err) => {
            println!("Could not interpret nav graph file name {nav_path:?}: {err:?}");
            return std::ptr::null_mut();
        }
    };
    let nav_f = match open_file(&nav_filename) {
        Some(f) => f,
        None => {
            println!("Could not open {nav_filename}, file not found");
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
            println!("Could not interpret agent file name {agent_path:?}: {err:?}");
            return std::ptr::null_mut();
        }
    };

    let agent_f = match open_file(&agent_filename) {
        Some(f) => f,
        None => {
            println!("Could not open {agent_filename}: file not found");
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

    for (i, ss) in sim_config.source_sinks.iter().enumerate() {
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

        // TODO(MXG): Let users configure the prefix
        let prefix = "source_sink_".to_owned() + &i.to_string() + "_";
        crowd_sim.add_source_sink(Arc::new(SourceSink {
            source,
            orientation: ss.orientation,
            prefix: prefix.to_owned(),
            model: ss.model.clone(),
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
            start, agent.orientation, name, &agent.model, agent.goal_radius,
            high_level_planner, local_planner, agent.avoidance.eyesight_range,
        ) {
            Ok(id) => id,
            Err(err) => {
                println!("Failed to add persistent agent: {err:?}");
                return std::ptr::null_mut();
            }
        };
        agent_map.insert(name.clone(), id);
    }

    let mut robot_map: HashMap<String, Option<AgentId>> = HashMap::new();
    for name in &sim_config.robots {
        robot_map.insert(name.clone(), None);
    }

    let event_listener = Arc::new(Mutex::new(
        Callbacks { system, spawn, moving, idle, reached }));
    crowd_sim.add_event_listener(event_listener.clone());

    Box::into_raw(Box::new(SimulationBinding
    {
        crowd_sim,
        agent_map,
        robot_map,
        location_map: locations,
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
            x: agent.position.x,
            y: agent.position.y,
            yaw: agent.orientation,
            visible: 1
        };
    }
    return Position{x: 0.0, y: 0.0, yaw: 0.0, visible: -1};
}

#[no_mangle]
pub extern "C" fn crowdsim_request_goal(
    ptr: *mut SimulationBinding,
    agent_id: u64,
    location: *const c_char,
) -> i64 {
    let location = match unsafe { CStr::from_ptr(location) }.to_str() {
        Ok(s) => s,
        Err(err) => {
            println!("Could not interpret location {location:?}: {err:?}");
            return -1;
        }
    };

    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    let location = match sim_binding.location_map.get(location) {
        Some(location) => location,
        None => {
            println!("Could not find requested location [{location}] in the nav graph");
            return -1;
        }
    };

    match sim_binding.crowd_sim.persistent_agent_request(
        agent_id as usize, *location,
    ) {
        Ok(r) => r as i64,
        Err(err) => {
            println!("Failed to request goal [{location}]: {err:?}");
            return -1;
        }
    }
}

#[no_mangle]
pub extern "C" fn crowdsim_run(
    ptr: *mut SimulationBinding,
    dt: c_double)
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

#[no_mangle]
pub extern "C" fn crowdsim_get_robot_id(
    ptr: *mut SimulationBinding,
    name: *const c_char,
) -> i64 {
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    let name = match unsafe { CStr::from_ptr(name) }.to_str() {
        Ok(s) => s,
        Err(err) => {
            println!("Could not interpret robot name {name:?}: {err:?}");
            return -1;
        }
    };
    match sim_binding.robot_map.get(name) {
        Some(opt_id) => {
            match opt_id {
                Some(id) => *id as i64,
                None => {
                    let id = match sim_binding.crowd_sim.add_obstacle(Vec2f::zeros(), 0.0) {
                        Ok(id) => id,
                        Err(err) => {
                            println!("Failed to add obstacle: {err:?}");
                            return -1;
                        }
                    };
                    sim_binding.robot_map.insert(name.to_owned(), Some(id));
                    id as i64
                }
            }
        }
        None => -1,
    }
}

#[no_mangle]
pub extern "C" fn crowdsim_update_robot_position(
    ptr: *mut SimulationBinding,
    robot_id: u64,
    position: Position,
) {
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    if let Err(err) = sim_binding.crowd_sim.update_obstacle(
        robot_id as usize, position.into()
    ) {
        println!("Error updating robot {robot_id}: {err}");
    }
}
