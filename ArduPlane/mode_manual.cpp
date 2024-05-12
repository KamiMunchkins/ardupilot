#include "mode.h"
#include "Plane.h"

#include <GCS_MAVLink/GCS_MAVLink.h>
uint32_t debugTimestamp5 = 0;
uint32_t debugTimestamp6 = 0;
#define LOG_PERIOD 3000

void ModeManual::update()
{
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.roll_in_expo(false));
    // SBL2 custom code
    plane.virtualElevator = plane.pitch_in_expo(false);
    plane.flushElevatorMixing(false);
    // SBL2 cp 2
    uint32_t nowDebug = AP_HAL::millis();
    if(nowDebug - debugTimestamp5 > LOG_PERIOD) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "main pitch_out %.2f", plane.virtualElevator);
        debugTimestamp5 = nowDebug;
    }
    // SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.pitch_in_expo(false));
    output_rudder_and_steering(plane.rudder_in_expo(false));
    float throttle = plane.get_throttle_input(true);


#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.option_is_set(QuadPlane::OPTION::IDLE_GOV_MANUAL)) {
        // for quadplanes it can be useful to run the idle governor in MANUAL mode
        // as it prevents the VTOL motors from running
        int8_t min_throttle = plane.aparm.throttle_min.get();

        // apply idle governor
#if AP_ICENGINE_ENABLED
        plane.g2.ice_control.update_idle_governor(min_throttle);
#endif
        throttle = MAX(throttle, min_throttle);
    }
#endif
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);

    plane.nav_roll_cd = ahrs.roll_sensor;
    plane.nav_pitch_cd = ahrs.pitch_sensor;
}

void ModeManual::run()
{
    reset_controllers();
}
