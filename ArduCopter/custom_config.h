
#ifndef CUSTOM_CONFIG_H
#define CUSTOM_CONFIG_H

// this flag can be used generally
#define FORCE_6DOF_ATTITUDE_CONTROLLER false
// set this to false when not in use
#define LATERAL_MOTORS_CONFIG4 false
// set this to false when not in use
#define LIFTING_MOTORS_REVERSIBLE false
#define MOT_SPIN_NEUTRAL 1500
// TODO maybe replace with get_pwm_output_min
#define MOT_SPIN_MIN 1000

// set to 1 to effectively disable this feature; This was added as a work in
// progress for trying to help stabilize a high rotational inertia aircraft. I
// think it's unnecessary because PID tuning should account for all concerns,
// but Larry wanted this. Might pick up again.
#define ACRO_YAW_GAIN 1.0

// feature that kills the motors on radio disconnect.
#define FAILSAFE_KILL_MOTORS true

// for GPS health check removal
#define INDOOR_AIRCRAFT false

// set to 0 to disable, 1 to enable. This is for taking out differential yaw
// control of all multicopter designs in order to allow us to control yaw
// separately with a tail fin.
#define DIFF_YAW_ENABLED 0
#define EXTERNAL_YAW_CONTROL (DIFF_YAW_ENABLED == 0)

// this is for if we're using indoor flight with no compass, but it was
// determined this is not necessary.
#define VISODOM_PRIMARY_COMPASS false

// NOT YET TESTED; this disables the assumption that optical flow and range
// finder rotate along with the flight controller. Useful for our custom
// airfame.
#define DISABLE_SENSOR_ROTATION false

#endif // CUSTOM_CONFIG_H
