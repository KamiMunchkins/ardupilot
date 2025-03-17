/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Motors6DOF.cpp - ArduSub motors library
 */

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Motors6DOF.h"
#include <GCS_MAVLink/GCS.h>

#include "../../ArduCopter/custom_config.h"

uint32_t lastLogTime6 = 0;
#define DEAD_BAND 0.05
#define LOG_PERIOD 3000
        /*
        bool debug = false;
        uint32_t now = AP_HAL::millis();
        if(now - lastLogTime6 > LOG_PERIOD) {
            lastLogTime6 = now;
            debug = true;
        }
        if(debug) {
                gcs().send_text(MAV_SEVERITY_INFO, "SBL forward_in was %.2f", _forward_in);
        }
        */

// SBL added this, everything else was hardcoded to 1500

// reversible quad motors for a balloon so we can for pitching down
// config4 vs config8. A config4 setup means we need to yaw by reversing the
// motors.


extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors6DOF::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: 1_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("1_DIRECTION", 1, AP_Motors6DOF, _motor_reverse[0], 1),

    // @Param: 2_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("2_DIRECTION", 2, AP_Motors6DOF, _motor_reverse[1], 1),

    // @Param: 3_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("3_DIRECTION", 3, AP_Motors6DOF, _motor_reverse[2], 1),

    // @Param: 4_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("4_DIRECTION", 4, AP_Motors6DOF, _motor_reverse[3], 1),

    // @Param: 5_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("5_DIRECTION", 5, AP_Motors6DOF, _motor_reverse[4], 1),

    // @Param: 6_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("6_DIRECTION", 6, AP_Motors6DOF, _motor_reverse[5], 1),

    // @Param: 7_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("7_DIRECTION", 7, AP_Motors6DOF, _motor_reverse[6], 1),

    // @Param: 8_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("8_DIRECTION", 8, AP_Motors6DOF, _motor_reverse[7], 1),

    // @Param: FV_CPLNG_K
    // @DisplayName: Forward/vertical to pitch decoupling factor
    // @Description: Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal
    // @Range: 0.0 1.5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FV_CPLNG_K", 9, AP_Motors6DOF, _forwardVerticalCouplingFactor, 1.0),

    // @Param: 9_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("9_DIRECTION", 10, AP_Motors6DOF, _motor_reverse[8], 1),

    // @Param: 10_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standagd
    AP_GROUPINFO("10_DIRECTION", 11, AP_Motors6DOF, _motor_reverse[9], 1),

    // @Param: 11_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("11_DIRECTION", 12, AP_Motors6DOF, _motor_reverse[10], 1),

    // @Param: 12_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("12_DIRECTION", 13, AP_Motors6DOF, _motor_reverse[11], 1),

    AP_GROUPEND
};

bool AP_Motors6DOF::init(uint8_t expected_num_motors) {
    //gcs().send_text(MAV_SEVERITY_INFO,"SBL inside custom init func, expected motors %d", expected_num_motors);

    // SBL hard-coded
    int wantMotors = 12;
    if(LATERAL_MOTORS_CONFIG4) {
      wantMotors = 8;
    }
    uint8_t num_motors = 0;
    for(uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }
    set_initialised_ok(wantMotors == num_motors);

    if (!initialised_ok()) {
        //gcs().send_text(MAV_SEVERITY_WARNING,"SBL init FAIL");
        return false;
    }
    set_update_rate(_speed_hz);
    //gcs().send_text(MAV_SEVERITY_WARNING,"SBL init SUCCESS");

    return true;
}

void AP_Motors6DOF::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok(false);
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    int wantMotors = 12;
    if(LATERAL_MOTORS_CONFIG4) {
      wantMotors = 8;
    }
    for(int i=0; i < wantMotors; i++) {
        _motor_reverse[i].set_and_save(1);
    }
    _frame_class_string = "EBlimp Custom";
    // 1, what should the throttle factor be for all of these? I believe 1
    // add_motor_raw_6dof(motNum, roll, pitch, yaw, throttle, forward, lateral, testOrder);
    float noForward = 0;
    float noLateral = 0;
    float noRoll = 0;
    float noPitch = 0;
    float noYaw = 0;

    // these yaw factors tested and verified
    float yawFactorCCW = -1.0;
    float yawFactorCW = 1.0;
    //gcs().send_text(MAV_SEVERITY_WARNING,"SBL CUSTOM MOTOR SETUP");

    // SBL CUSTOM MOTOR FRAME
    // top left
    // should be spinning CCW

    float pitchDown = 1.0;
    float rollRight = 1.0;
    float forward = 1.0;
    float lateral = 1.0;
    // top left
    add_motor_raw_6dof(AP_MOTORS_MOT_1, rollRight, noPitch, yawFactorCCW, 1.0, noForward, noLateral, 1);
    // top right
    add_motor_raw_6dof(AP_MOTORS_MOT_2, -rollRight, noPitch, yawFactorCW, 1.0, noForward, noLateral, 2);
    // bottom right
    add_motor_raw_6dof(AP_MOTORS_MOT_3, -rollRight, noPitch, yawFactorCCW, 1.0, noForward, noLateral, 3);
    // bottom left
    add_motor_raw_6dof(AP_MOTORS_MOT_4, rollRight, noPitch, yawFactorCW, 1.0, noForward, noLateral, 4);

    // back left
    add_motor_raw_6dof(AP_MOTORS_MOT_7, noRoll, pitchDown, noYaw, 0.0, forward, noLateral, 7);
    // back right
    add_motor_raw_6dof(AP_MOTORS_MOT_8, noRoll, pitchDown, noYaw, 0.0, forward, noLateral, 8);

    }
    set_initialised_ok(true);

    return;

    // hard coded config for supported frames
    switch ((sub_frame_t)frame_class) {
        //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
    case SUB_FRAME_BLUEROV1:
        _frame_class_string = "BLUEROV1";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     -0.5f,          0.5f,           0,              0.45f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0.5f,           0.5f,           0,              0.45f,              0,                  0,              4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              -1.0f,          0,              1.0f,               0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -0.25f,         0,              0,              0,                  0,                  1.0f,           6);
        break;

    case SUB_FRAME_VECTORED_6DOF_90DEG:
        _frame_class_string = "VECTORED_6DOF_90DEG";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     1.0f,           1.0f,           0,              1.0f,               0,                  0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,          0,              1.0f,               0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              0,              0,                  0,                  1.0f,           4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              0,              0,              0,                  0,                  1.0f,           5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          1.0f,           0,              1.0f,               0,                  0,              6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     0,              0,              -1.0f,          0,                  1.0f,               0,              7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     -1.0f,          -1.0f,          0,              1.0f,               0,                  0,              8);
        break;

    case SUB_FRAME_VECTORED_6DOF:
        _frame_class_string = "VECTORED_6DOF";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              1.0f,           0,                  -1.0f,              1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,          2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              -1.0f,          0,                  1.0f,               1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              1.0f,           0,                  1.0f,               -1.0f,          4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           -1.0f,          0,              -1.0f,              0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          -1.0f,          0,              -1.0f,              0,                  0,              6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     1.0f,           1.0f,           0,              -1.0f,              0,                  0,              7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     -1.0f,          1.0f,           0,              -1.0f,              0,                  0,              8);
        break;

    case SUB_FRAME_VECTORED:
        _frame_class_string = "VECTORED";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              1.0f,           0,                  -1.0f,              1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,          2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              -1.0f,          0,                  1.0f,               1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              1.0f,           0,                  1.0f,               -1.0f,          4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           0,              0,              -1.0f,              0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          0,              0,              -1.0f,              0,                  0,              6);
        break;

    case SUB_FRAME_CUSTOM:
        // Put your custom motor setup here
        //break;

    case SUB_FRAME_SIMPLEROV_3:
        _frame_class_string = "SIMPLEROV_3";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              0,              -1.0f,              0,                  0,              3);
        break;
    case SUB_FRAME_SIMPLEROV_4:
    case SUB_FRAME_SIMPLEROV_5:
    default:
        _frame_class_string = "DEFAULT";
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           0,              0,              -1.0f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          0,              0,              -1.0f,              0,                  0,              4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              0,              0,              0,                  0,                  1.0f,           5);
        break;
    }
}

void AP_Motors6DOF::add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float lat_fac, uint8_t testing_order)
{
    //Parent takes care of enabling output and setting up masks
    add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);

    //These are additional parameters for an ROV
    _throttle_factor[motor_num] = throttle_fac;
    _forward_factor[motor_num] = forward_fac;
    _lateral_factor[motor_num] = lat_fac;
}

// output_min - sends minimum values out to the motors
void AP_Motors6DOF::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll = true;
    limit.pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    // ToDo find a field to store the minimum pwm instead of hard coding 1500
    // SBL redundant code
    // NOTE rc_write() usage vs motor_out[i] = ; this tripped me up earlier
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if(i < 4) {
              if(LIFTING_MOTORS_REVERSIBLE) {
                rc_write(i, MOT_SPIN_NEUTRAL);
              } else {
                rc_write(i, MOT_SPIN_MIN);
              }
            } else {
              if(LATERAL_MOTORS_CONFIG4) {
                rc_write(i, MOT_SPIN_NEUTRAL);
              } else {
                rc_write(i, MOT_SPIN_MIN);
              }
            }
        }
    }
}

int16_t AP_Motors6DOF::calc_thrust_to_pwm(float thrust_in, bool reversible) const
{
    if(!reversible) {
        // SBL modified this.
        int16_t minPwm = get_pwm_output_min();
        if(thrust_in <= 0) {
            return minPwm;
        }
        int16_t interp_range = get_pwm_output_max() - get_pwm_output_min();
        return (thrust_in * interp_range) + minPwm;
    }
    if(abs(thrust_in) <= DEAD_BAND) {
        thrust_in = 0;
    }
    int16_t range_up = get_pwm_output_max() - MOT_SPIN_NEUTRAL;
    int16_t range_down = MOT_SPIN_NEUTRAL - get_pwm_output_min();
    return MOT_SPIN_NEUTRAL + thrust_in * (thrust_in > 0 ? range_up : range_down);
}

void AP_Motors6DOF::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        // sends minimum values out to the motors
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            // SBL redundant code
            if (motor_enabled[i]) {
                if(i < 4) {
                  if(LIFTING_MOTORS_REVERSIBLE) {
                    motor_out[i] = MOT_SPIN_NEUTRAL;
                  } else {
                    motor_out[i] = MOT_SPIN_MIN;
                  }
                } else {
                  if(LATERAL_MOTORS_CONFIG4) {
                    motor_out[i] = MOT_SPIN_NEUTRAL;
                  } else {
                    motor_out[i] = MOT_SPIN_MIN;
                  }
                }
            }
        }
        break;
    case SpoolState::GROUND_IDLE:
        // sends output to motors when armed but not flying
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            // SBL redundant code
            if (motor_enabled[i]) {
                if(i < 4) {
                  if(LIFTING_MOTORS_REVERSIBLE) {
                    motor_out[i] = MOT_SPIN_NEUTRAL;
                  } else {
                    motor_out[i] = MOT_SPIN_MIN;
                  }
                } else {
                  if(LATERAL_MOTORS_CONFIG4) {
                    motor_out[i] = MOT_SPIN_NEUTRAL;
                  } else {
                    motor_out[i] = MOT_SPIN_MIN;
                  }
                }
            }
        }
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                if(i < 4) {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i], LIFTING_MOTORS_REVERSIBLE);
                } else {
                    motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i], LATERAL_MOTORS_CONFIG4);
                }
            }
        }
        break;
    }

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, motor_out[i]);
        }
    }
}

float AP_Motors6DOF::get_current_limit_max_throttle()
{
    return 1.0f;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_Motors6DOF::output_armed_stabilizing()
{
    // SBL output case 3
    if(false) {
        // SBL DELETE THESE CASES
        if ((sub_frame_t)_active_frame_class == SUB_FRAME_VECTORED) {
            output_armed_stabilizing_vectored();
        } else if ((sub_frame_t)_active_frame_class == SUB_FRAME_VECTORED_6DOF) {
            output_armed_stabilizing_vectored_6dof();
        }
    } else {
        uint8_t i;                          // general purpose counter
        float   roll_thrust;                // roll thrust input value, +/- 1.0
        float   pitch_thrust;               // pitch thrust input value, +/- 1.0
        float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
        float   throttle_thrust;            // throttle thrust input value, +/- 1.0
        // NOTE forward and lateral thrust are actually +/- 0.5
        float   forward_thrust;             // forward thrust input value, +/- 1.0
        float   lateral_thrust;             // lateral thrust input value, +/- 1.0

        roll_thrust = (_roll_in + _roll_in_ff);
        pitch_thrust = (_pitch_in + _pitch_in_ff);
        yaw_thrust = (_yaw_in + _yaw_in_ff);
        if(LIFTING_MOTORS_REVERSIBLE) {
            throttle_thrust = get_throttle_bidirectional();
        } else {
            const float compensation_gain = thr_lin.get_compensation_gain(); // compensation for battery voltage and altitude
            throttle_thrust = get_throttle() * compensation_gain;
        }
        forward_thrust = _forward_in;
        lateral_thrust = _lateral_in;

        float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
        float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

        // initialize limits flags
        limit.roll = false;
        limit.pitch = false;
        limit.yaw = false;
        limit.throttle_lower = false;
        limit.throttle_upper = false;

        // sanity check throttle is above zero and below current limited throttle
        if(LIFTING_MOTORS_REVERSIBLE) {
            if (throttle_thrust <= -_throttle_thrust_max) {
                throttle_thrust = -_throttle_thrust_max;
                limit.throttle_lower = true;
            }
        } else {
            // SBL MODIFYING THIS CASE BECAUSE FOR NON-REVERSIBLE MOTORS
            if (throttle_thrust <= 0) {
                throttle_thrust = 0;
                limit.throttle_lower = true;
            }
            if (throttle_thrust >= _throttle_thrust_max) {
                throttle_thrust = _throttle_thrust_max;
                limit.throttle_upper = true;
            }
        }

        // SBL hard-coded
        // This is to elminate motor saturation. Max forward_in is 0.5,
        // so limiting to 0.5 prevents some motors getting saturated while others are not.
        if(yaw_thrust > 0.5) {
            yaw_thrust = 0.5;
            limit.yaw = true;
        } else if (yaw_thrust < -0.5) {
            yaw_thrust = -0.5;
            limit.yaw = true;
        }


        // calculate roll, pitch and yaw for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rpy_out[i] = roll_thrust * _roll_factor[i] +
                             pitch_thrust * _pitch_factor[i] +
                             yaw_thrust * _yaw_factor[i];

            }
        }
        // SBL need to modify here. What's happening is that we have poor yaw control because some motors on opposing side of the
        // running motors are negative, and therefore yaw has to overcome this negative value before they even activate.
        // 2 cases to account for:
        // 1. Yaw isn't activating if one side of motors is running
        //    This can be fixed by setting forward_thrust to 0 if it is negative when multiplied by forward factor
        // 2. Forward and lateral max out at 1500 PWM.
        //


        // calculate linear command for each motor
        // linear factors should be 0.0 or 1.0 for now
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                float local_forward_thrust = forward_thrust;
                float local_lateral_thrust = lateral_thrust;
                if(!LATERAL_MOTORS_CONFIG4) {
                    if(local_forward_thrust * _forward_factor[i] < 0) {
                        local_forward_thrust = 0;
                    }
                    if (local_lateral_thrust * _lateral_factor[i] < 0) {
                        local_lateral_thrust = 0;
                    }
                }
                linear_out[i] = throttle_thrust * _throttle_factor[i] +
                                local_forward_thrust * _forward_factor[i] +
                                local_lateral_thrust * _lateral_factor[i];
            }
        }

        // Calculate final output for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]),-1.0f,1.0f);
            }
        }
    }

    const AP_BattMonitor &battery = AP::battery();

	// Current limiting
    float _batt_current;
    if (_batt_current_max <= 0.0f || !battery.current_amps(_batt_current)) {
        return;
    }

    float _batt_current_delta = _batt_current - _batt_current_last;

    float _current_change_rate = _batt_current_delta / _dt;

    float predicted_current = _batt_current + (_current_change_rate * _dt * 5);

    float batt_current_ratio = _batt_current / _batt_current_max;

    float predicted_current_ratio = predicted_current / _batt_current_max;
    _batt_current_last = _batt_current;

    if (predicted_current > _batt_current_max * 1.5f) {
        batt_current_ratio = 2.5f;
    } else if (_batt_current < _batt_current_max && predicted_current > _batt_current_max) {
        batt_current_ratio = predicted_current_ratio;
    }
    _output_limited += (_dt / (_dt + _batt_current_time_constant)) * (1 - batt_current_ratio);

    _output_limited = constrain_float(_output_limited, 0.0f, 1.0f);

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] *= _output_limited;
        }
    }
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_Motors6DOF::output_armed_stabilizing_vectored()
{
    // SBL output case 1 UNUSED
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0

    roll_thrust = (_roll_in + _roll_in_ff);
    pitch_thrust = (_pitch_in + _pitch_in_ff);
    yaw_thrust = (_yaw_in + _yaw_in_ff);
    throttle_thrust = get_throttle_bidirectional();
    forward_thrust = _forward_in;
    lateral_thrust = _lateral_in;

    float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

    // initialize limits flags
    limit.roll= false;
    limit.pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= -_throttle_thrust_max) {
        throttle_thrust = -_throttle_thrust_max;
        limit.throttle_lower = true;
    }

    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         yaw_thrust * _yaw_factor[i];
        }
    }

    float forward_coupling_limit = 1-_forwardVerticalCouplingFactor*float(fabsf(throttle_thrust));
    if (forward_coupling_limit < 0) {
        forward_coupling_limit = 0;
    }
    int8_t forward_coupling_direction[] = {-1,-1,1,1,0,0,0,0,0,0,0,0};

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

            float forward_thrust_limited = forward_thrust;

            // The following statements decouple forward/vertical hydrodynamic coupling on
            // vectored ROVs. This is done by limiting the maximum output of the "rear" vectored
            // thruster (where "rear" depends on direction of travel).
            if (!is_zero(forward_thrust_limited)) {
                if ((forward_thrust < 0) == (forward_coupling_direction[i] < 0) && forward_coupling_direction[i] != 0) {
                    forward_thrust_limited = constrain_float(forward_thrust, -forward_coupling_limit, forward_coupling_limit);
                }
            }

            linear_out[i] = throttle_thrust * _throttle_factor[i] +
                            forward_thrust_limited * _forward_factor[i] +
                            lateral_thrust * _lateral_factor[i];
        }
    }

    // Calculate final output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]), -1.0f, 1.0f);
        }
    }
}

// Band Aid fix for motor normalization issues.
// TODO: find a global solution for managing saturation that works for all vehicles
void AP_Motors6DOF::output_armed_stabilizing_vectored_6dof()
{
    // SBL output case 2 UNUSED
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0

    roll_thrust = (_roll_in + _roll_in_ff);
    pitch_thrust = (_pitch_in + _pitch_in_ff);
    yaw_thrust = (_yaw_in + _yaw_in_ff);
    if(LIFTING_MOTORS_REVERSIBLE) {
        throttle_thrust = get_throttle_bidirectional();
    } else {
        // SBL modified here
        const float compensation_gain = thr_lin.get_compensation_gain(); // compensation for battery voltage and altitude
        throttle_thrust = get_throttle() * compensation_gain;
    }
    forward_thrust = _forward_in;
    lateral_thrust = _lateral_in;

    float rpt_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float yfl_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
    float rpt_max;
    float yfl_max;

    // initialize limits flags
    limit.roll = false;
    limit.pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= -_throttle_thrust_max) {
        throttle_thrust = -_throttle_thrust_max;
        limit.throttle_lower = true;
    }

    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // calculate roll, pitch and Throttle for each motor (only used by vertical thrusters)
    rpt_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpt_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         throttle_thrust * _throttle_factor[i];
            if (fabsf(rpt_out[i]) > rpt_max) {
                rpt_max = fabsf(rpt_out[i]);
            }
        }
    }

    // calculate linear/yaw command for each motor (only used for translational thrusters)
    // linear factors should be 0.0 or 1.0 for now
    yfl_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            yfl_out[i] = yaw_thrust * _yaw_factor[i] +
                         forward_thrust * _forward_factor[i] +
                         lateral_thrust * _lateral_factor[i];
            if (fabsf(yfl_out[i]) > yfl_max) {
                yfl_max = fabsf(yfl_out[i]);
            }
        }
    }

    // Calculate final output for each motor and normalize if necessary
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);
        }
    }
}

Vector3f AP_Motors6DOF::get_motor_angular_factors(int motor_number) {
     if (motor_number < 0 || motor_number >= AP_MOTORS_MAX_NUM_MOTORS) {
        return Vector3f(0,0,0);
    }
    return Vector3f(_roll_factor[motor_number], _pitch_factor[motor_number], _yaw_factor[motor_number]);
}

bool AP_Motors6DOF::motor_is_enabled(int motor_number) {
    if (motor_number < 0 || motor_number >= AP_MOTORS_MAX_NUM_MOTORS) {
        return false;
    }
    return motor_enabled[motor_number];
}

bool AP_Motors6DOF::set_reversed(int motor_number, bool reversed) {
    if (motor_number < 0 || motor_number >= AP_MOTORS_MAX_NUM_MOTORS) {
        return false;
    }
    if (reversed) {
        _motor_reverse[motor_number].set_and_save(-1);
    } else {
        _motor_reverse[motor_number].set_and_save(1);
    }
    return true;
}
