# How to convert some code of the ackermann sterring plugin to Rust

This tutorial explains how to convert some code of the [ackermann plugin](https://github.com/gazebosim/gz-sim/tree/gz-sim7/src/systems/ackermann_steering)
to Rust. This tutorial will focus in which parts
you can move to Rust. As a general idea, everything which is not related with the ECM.

We are going to create a new Gazebo package with the following structure

```
rust_hello_world
├── rust                        Rust files
│    ├── src                    Rust code
│    ├── rust_interface.h       Rust bindings
│    ├── AckermannSteering.cc   Gazebo plugin implementation
│    ├── AckermannSteering.hh   Gazebo plugin header
├── README.md                   Description of the project.
└── CMakeLists.txt              CMake build script.
```

## Create the workspace

Create the following workspace:

```bash
mkdir ~/rust_ws/src -p
cd ~/rust_ws
```

## Code

We are going to move the logic in the ackermann plugin to Rust. Let's include in
the `rust_interface.h` the struct with some of the relevant fields.
You can remove all this fields from the AckermannSteering original Gazebo plugin.
We are not exposing all the available attributes of the struct to C, because this will
allow to explain how to use getters.

```c
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
```

We will need to include the *constructor* and *destructor*:

```c
extern "C" simulation_binding_t * simulation_binding_new(
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
```

we will need to include this line in the `Configure` method in `AckermannSteering.cc`:

```c
...
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
...

AckermannSteering::~AckermannSteering()
{
  simulation_binding_free(this->dataPtr->simulationBindings);
}
```

## Some internal Rust code

The `SpeedLimiter` class is not defined in Rust (it's available in `gz-math`), we need to include this in the `lib.rs`

```rust
pub struct SpeedLimiter {
    min_velocity: f64,
    max_velocity: f64,
    min_acceleration: f64,
    max_acceleration: f64,
    min_jerk: f64,
    max_jerk: f64
}

impl SpeedLimiter {
    // Constructs a new instance of [`Second`].
    // Note this is an associated function - no self.
    pub fn new(
        min_velocity: f64,
        max_velocity: f64,
        min_acceleration: f64,
        max_acceleration: f64,
        min_jerk: f64,
        max_jerk: f64) -> Self {
        Self {
            min_velocity,
            max_velocity,
            min_acceleration,
            max_acceleration,
            min_jerk,
            max_jerk
        }
    }

    pub fn limit(&mut self, vel: &mut f64, prev_vel: f64, prev_prev_vel: f64, dt: f64) -> f64
    {
        let v_unclamped = *vel;

        self.limit_jerk(vel, prev_vel, prev_prev_vel, dt);
        self.limit_acceleration(vel, prev_vel, dt);
        self.limit_velocity(vel);

        return *vel - v_unclamped;
    }

    pub fn limit_velocity(&mut self, vel: &mut f64) -> f64
    {
        let v_unclamped = *vel;
        let vel_clamp = clamp(
          v_unclamped, self.min_velocity, self.max_velocity);

        *vel = vel_clamp;
        return vel_clamp - v_unclamped;
    }

    pub fn limit_acceleration(&mut self, vel: &mut f64, prev_vel: f64, dt: f64) -> f64
    {
        let v_unclamped = *vel;

        let dt_sec = dt;

        if equal(dt_sec, 0.0, 1e-6)
        {
          return 0.0;
        }

        let acc_unclamped = (v_unclamped - prev_vel) / dt_sec;

        let acc_clamped = clamp(acc_unclamped,
            self.min_acceleration, self.max_acceleration);

        *vel = prev_vel + acc_clamped * dt_sec;

        return *vel - v_unclamped;
    }

    pub fn limit_jerk(&mut self, vel: &mut f64, prev_vel: f64, prev_prev_vel: f64, dt: f64) -> f64
    {
        let v_unclamped = *vel;

        let dt_sec = dt;

        if equal(dt_sec, 0.0, 1e-6)
        {
          return 0.0;
        }

        let acc_unclamped  = (*vel  - prev_vel) / dt_sec;
        let acc_prev = (prev_vel - prev_prev_vel) / dt_sec;
        let jerk_unclamped = (acc_unclamped - acc_prev) / dt_sec;

        let jerk_clamped = clamp(jerk_unclamped,
            self.min_jerk, self.max_jerk);

        let acc_clamped = acc_prev + jerk_clamped * dt_sec;

        *vel = prev_vel + acc_clamped * dt_sec;

        return *vel - v_unclamped;
    }
}
```

Some other functions are relevant too:

```rust
fn clamp(v: f64, min: f64, max: f64) -> f64
{
    return f64::max(f64::min(v, max), min)
}

fn equal(a: f64, b: f64, epsilon: f64) -> bool
{
    let diff = f64::abs(a-b);
    return diff <= epsilon;
}

fn normalize_angle(angle: f64) -> f64
{
    return f64::atan2(f64::sin(angle), f64::cos(angle));
}
```

## Relevant functions

The following method will calculate the speed of the wheels. We need to include as
an argument the struct that is shared between Rust and Gazebo.

```c
extern "C" void update_velocity(simulation_binding_t *t,
    double& linear, double& angular,
    double leftSteeringPos, double rightSteeringPos,
    double dt);
```

Use this code in the `UpdateVelocity` method

```cpp
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
```

Same idea with the odometry:

```c
extern "C" bool calculate_odometry(simulation_binding_t *t,
    double leftPos, double rightPos,
    double leftSteeringPos, double rightSteeringPos,
    double simTime);
```

```cpp
void AckermannSteeringPrivate::UpdateOdometry(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  ...
  if(!calculate_odometry(this->simulationBindings,
                     leftPos->Data()[0], rightPos->Data()[0],
                     leftSteeringPos->Data()[0], rightSteeringPos->Data()[0],
                     std::chrono::duration<double>(_info.simTime).count()))
  {
    return;
  }
  ...
}
```

Finally we defined some getter to get the relevant info from the struct

```c
extern "C" double get_left_joint_speed(simulation_binding_t *t);
extern "C" double get_right_joint_speed(simulation_binding_t *t);
extern "C" double get_left_steering_speed(simulation_binding_t *t);
extern "C" double get_right_steering_speed(simulation_binding_t *t);
extern "C" double get_odom_x(simulation_binding_t *t);
extern "C" double get_odom_y(simulation_binding_t *t);
extern "C" double get_odom_yaw(simulation_binding_t *t);
extern "C" double get_odom_linear(simulation_binding_t *t);
extern "C" double get_odom_angular(simulation_binding_t *t);
```

We should use these calls in the Gazebo plugins instead of the attribute that we have already removed.

# More details
If you want to see more details about this examples please review these files:

- [AckermannSteering.cc](tutorial/ackermann_plugin/src/AckermannSteering.cc)
- [AckermannSteering.hh](tutorial/ackermann_plugin/src/AckermannSteering.hh)
- [rust_interface.h](tutorial/ackermann_plugin/src/rust_interface.h)
- [Rust code](/home/ahcorde/tmp/rmf_crowdsim_gz_integration/tutorial/ackermann_plugin/src/rust/src/lib.rs)
