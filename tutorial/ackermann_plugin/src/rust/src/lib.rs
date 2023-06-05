
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


#[repr(C)]
pub struct SimulationBinding
{
    /// \brief Calculated speed of left wheel joint(s)
    left_joint_speed: f64,

    /// \brief Calculated speed of right wheel joint(s)
    right_joint_speed: f64,

    /// \brief Calculated speed of left joint
    left_steering_joint_speed: f64,

    /// \brief Calculated speed of right joint
    right_steering_joint_speed: f64,

    /// \brief Linear velocity limiter.
    limiter_lin: SpeedLimiter,

    /// \brief Angular velocity limiter.
    limiter_ang: SpeedLimiter,

    /// \brief Distance between left and right wheels
    wheel_separation: f64,

    /// \brief Distance between left and right wheel kingpins
    kingpin_width: f64,

    /// \brief Distance between front and back wheels
    wheel_base: f64,

    /// \brief Maximum turning angle to limit steering to
    steering_limit: f64,

    /// \brief Wheel radius
    wheel_radius: f64,

    last_0_cmd_linear: f64,

    last_0_cmd_angular: f64,

    last_1_cmd_linear: f64,

    last_1_cmd_angular: f64,

    /// \brief Odometry X value
    odom_x: f64,

    /// \brief Odometry Y value
    odom_y: f64,

    /// \brief Odometry yaw value
    odom_yaw: f64,

    odom_linear_velocity: f64,

    odom_angular_velocity: f64,

    /// \brief Odometry old left value
    odom_old_left: f64,

    /// \brief Odometry old right value
    odom_old_right: f64,

    /// \brief Last sim time odom was published.
    last_odom_pub_time: f64,

    /// \brief Odometry last time value
    last_odom_time: f64,

    /// \brief Update period calculated from <odom__publish_frequency>.
    odom_pub_period: f64
}

#[no_mangle]
pub extern "C" fn simulation_binding_new(
    wheel_separation: f64,
    kingpin_width: f64,
    wheel_base: f64,
    steering_limit: f64,
    wheel_radius: f64,
    min_vel: f64,
    max_vel: f64,
    min_accel: f64,
    max_accel: f64,
    min_jerk: f64,
    max_jerk: f64,
    odom_pub_period: f64
) -> *mut SimulationBinding
{
    println!("odom_pub_period {}", odom_pub_period);
    Box::into_raw(Box::new(SimulationBinding
    {
        left_joint_speed: 0.0,
        right_joint_speed: 0.0,
        left_steering_joint_speed: 0.0,
        right_steering_joint_speed: 0.0,
        limiter_lin: SpeedLimiter::new(min_vel, max_vel, min_accel, max_accel, min_jerk, max_jerk),
        limiter_ang: SpeedLimiter::new(min_vel, max_vel, min_accel, max_accel, min_jerk, max_jerk),
        wheel_separation: wheel_separation,
        kingpin_width: kingpin_width,
        wheel_base: wheel_base,
        steering_limit: steering_limit,
        wheel_radius: wheel_radius,
        last_0_cmd_linear: 0.0,
        last_0_cmd_angular: 0.0,
        last_1_cmd_linear: 0.0,
        last_1_cmd_angular: 0.0,
        odom_x: 0.0,
        odom_y: 0.0,
        odom_yaw: 0.0,
        odom_linear_velocity: 0.0,
        odom_angular_velocity: 0.0,
        odom_old_left: 0.0,
        odom_old_right: 0.0,
        last_odom_pub_time: 0.0,
        last_odom_time: 0.0,
        odom_pub_period: odom_pub_period
    }))
}

#[no_mangle]
pub extern "C" fn simulation_binding_free(ptr: *mut SimulationBinding)
{
    if ptr.is_null() {
        return;
    }
    unsafe {
        Box::from_raw(ptr);
    }
}

#[no_mangle]
pub extern "C" fn get_left_joint_speed(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.left_joint_speed;
}

#[no_mangle]
pub extern "C" fn get_right_joint_speed(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.right_joint_speed;
}

#[no_mangle]
pub extern "C" fn get_left_steering_speed(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.left_steering_joint_speed;
}

#[no_mangle]
pub extern "C" fn get_right_steering_speed(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.right_steering_joint_speed;
}

#[no_mangle]
pub extern "C" fn update_velocity(ptr: *mut SimulationBinding,
    linear: &mut f64, angular: &mut f64,
    left_steering_pos: f64, right_steering_pos: f64,
    dt: f64)
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    // Limit the target velocity if needed.
    sim_binding.limiter_lin.limit(
        linear, sim_binding.last_0_cmd_linear, sim_binding.last_1_cmd_linear, dt);
    sim_binding.limiter_ang.limit(
        angular, sim_binding.last_0_cmd_angular, sim_binding.last_1_cmd_angular, dt);

    sim_binding.last_1_cmd_linear = sim_binding.last_0_cmd_linear;
    sim_binding.last_1_cmd_angular = sim_binding.last_0_cmd_angular;
    sim_binding.last_0_cmd_linear = *linear;
    sim_binding.last_0_cmd_angular = *angular;

    // Convert the target velocities to joint velocities and angles
    let mut turning_radius = *linear / *angular;
    let minimum_turning_radius = sim_binding.wheel_base / f64::sin(sim_binding.steering_limit);
    if (turning_radius >= 0.0) && (turning_radius < minimum_turning_radius)
    {
        turning_radius = minimum_turning_radius;
    }
    if (turning_radius <= 0.0) && (turning_radius > -minimum_turning_radius)
    {
        turning_radius = -minimum_turning_radius;
    }
    // special case for angVel of zero
    if f64::abs(*angular) < 0.001
    {
        turning_radius = 1000000000.0;
    }

    let left_steering_joint_angle =
      f64::atan(sim_binding.wheel_base / (turning_radius - (sim_binding.kingpin_width / 2.0)));
    let right_steering_joint_angle =
      f64::atan(sim_binding.wheel_base / (turning_radius + (sim_binding.kingpin_width / 2.0)));
    let phi =f64:: atan(sim_binding.wheel_base / turning_radius);

    // Partially simulate a simple differential
    sim_binding.right_joint_speed =
      (*linear * (1.0 + (sim_binding.wheel_separation * f64::tan(phi)) /
                 (2.0 * sim_binding.wheel_base))) / sim_binding.wheel_radius;
    sim_binding.left_joint_speed =
      (*linear * (1.0 - (sim_binding.wheel_separation * f64::tan(phi)) /
                 (2.0 * sim_binding.wheel_base))) / sim_binding.wheel_radius;

    let left_delta = left_steering_joint_angle - left_steering_pos;
    let right_delta = right_steering_joint_angle - right_steering_pos;

    // Simple proportional control with a gain of 1
    // Adding programmable PID values might be a future feature.
    // Works as is for tested cases
    sim_binding.left_steering_joint_speed = left_delta;
    sim_binding.right_steering_joint_speed = right_delta;
}

#[no_mangle]
pub extern "C" fn get_odom_x(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.odom_x;
}

#[no_mangle]
pub extern "C" fn get_odom_y(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.odom_y;
}

#[no_mangle]
pub extern "C" fn get_odom_yaw(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.odom_yaw;
}

#[no_mangle]
pub extern "C" fn get_odom_linear(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.odom_linear_velocity;
}

#[no_mangle]
pub extern "C" fn get_odom_angular(ptr: *mut SimulationBinding) -> f64
{
    let sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };
    return sim_binding.odom_angular_velocity;
}

#[no_mangle]
pub extern "C" fn calculate_odometry(ptr: *mut SimulationBinding,
    left_pos: f64, right_pos: f64,
    left_steering_pos: f64, right_steering_pos: f64,
    sim_time: f64) -> bool
{
    let mut sim_binding = unsafe {
        assert!(!ptr.is_null());
        &mut *ptr
    };

    // Calculate the odometry
    let phi = 0.5 * (left_steering_pos + right_steering_pos);
    let radius = sim_binding.wheel_base / f64::tan(phi);
    let dist = 0.5 * sim_binding.wheel_radius *
        ((left_pos - sim_binding.odom_old_left) +
         (right_pos - sim_binding.odom_old_right));
    let delta_angle = dist / radius;
    sim_binding.odom_yaw += delta_angle;
    sim_binding.odom_yaw = normalize_angle(sim_binding.odom_yaw);
    sim_binding.odom_x += dist * f64::cos(sim_binding.odom_yaw);
    sim_binding.odom_y += dist * f64::sin(sim_binding.odom_yaw);
    let tdiff = sim_time - sim_binding.last_odom_time;
    sim_binding.odom_linear_velocity = dist / tdiff;
    sim_binding.odom_angular_velocity = delta_angle / tdiff;
    sim_binding.last_odom_time = sim_time;
    sim_binding.odom_old_left = left_pos;
    sim_binding.odom_old_right = right_pos;

    // Throttle odometry publishing
    let diff = sim_time - sim_binding.last_odom_pub_time;
    println!("sim_time {}", sim_time);
    println!("sim_binding.last_odom_pub_time {}", sim_binding.last_odom_pub_time);
    println!("diff {}", diff);
    if diff > 0.0 && diff < sim_binding.odom_pub_period
    {
      return false;
    }
    sim_binding.last_odom_pub_time = sim_time;
    return true;
}
