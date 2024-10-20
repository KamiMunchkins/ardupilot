
#ifndef CUSTOM_CONFIG_H
#define CUSTOM_CONFIG_H

#define LATERAL_MOTORS_CONFIG4 true
#define LIFTING_MOTORS_REVERSIBLE true
#define MOT_SPIN_NEUTRAL 1500
// TODO maybe replace with get_pwm_output_min
#define MOT_SPIN_MIN 1000

// TODO the better way of doing this is to updating tuning.cpp with my own custom behaviors
#define ACRO_YAW_GAIN_MIN 1.0
#define ACRO_YAW_GAIN_MAX 10.0
#define ACRO_YAW_GAIN_CHANNEL 6

#endif // CUSTOM_CONFIG_H
