# Gazebo Plugin Written in Rust: Crowd sim integration

![](doc/rustling_march.gif)
This repo contains my experiments with rust + gazebo.

You will need Gazebo Garden installed. To build this repo simply run:
```
colcon build
```

`corrosion-rs` and `cargo` will take care of the rest of the magic.

## C-API documentation
Complete documentation for the C-API  can be found [here](rusty_gz_plugin/src/rust_interface.h)

## Plugin documentations

The plugin can be loaded like so:
```
 <plugin
      filename="libRustySystem.so"
      name="rusty::RustySystem">
      <!-- TODO: use relative paths -->
    <path>/home/arjo/workspaces/chartsim/src/chart_sim_maps/maps/ward45/ward45.building.yaml</path>
    <sources>
        <source_sink>
            <rate>1</rate>
            <start>-23 -1 0</start>
            <waypoints>
              <waypoint>9 -3 0</waypoint>
            </waypoints>
        </source_sink>
    </sources>
</plugin>
```


### SDFormat Arguments
* `<agents>` - Path to a yaml file describing the agent behavior (see Agents File section below).
* `<nav>` - Path to a navigation graph (produced by traffic-editor or site-editor) that contains the waypoints used by the agents.

### Agents File

Example yaml file below

```
# Name of the level that contains agents.
# Only one floor is supported at the moment.
level: L1

# Descriptions of persistent agents in the world.
# Persistent agents are always in the world and can be commanded
# to move to designated locations at any time.
agents:
  # Unique name for the agent
  nurse_1:
    # Name of start location for the nurse
    start: NurseDesk
    # Model to use for the nurse
    model: Nurse
  elderly_patient_32:
    start: bed_32
    model: ElderlyPatient

# Descriptions of source-sink patterns.
# Source-Sinks will generate temporary agents that spawn with a parameterized
# uniform distribution, follow a sequence of waypoints, and then vanish.
source_sinks:
  - model: Visitor
    source: ward_45_entrance
    waypoints:
    - NurseDesk
    - bed_32
    - ward_45_entrance
    # average number of temporary agents spawned per second
    rate: 0.01
  - model: Orderly
    source: utility_room
    waypoints: [bed_32, bed_31, bed_30, bed_25, bed_24]
    rate: 0.005
```

More parameters can be used to customize the agent behaviors.
Those parameters can be found in the structs defined in [rusty_gz_plugin/src/rust_system/src/lib.rs](https://github.com/open-rmf/rmf_crowdsim_gz_integration/blob/master/rusty_gz_plugin/src/rust_system/src/lib.rs).
