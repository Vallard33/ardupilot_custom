#include "Sub.h"


bool ModeChad::init(bool ignore_checks) {

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Its CHAD time !");

    ////////////////// Stabilize /////////////

    position_control->set_pos_target_z_cm(0);
    sub.last_pilot_heading = ahrs.yaw_sensor;
    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void ModeChad::run()
{

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        ///////////// Stabilize ////////////////
        attitude_control->set_throttle_out(0,true,g.throttle_filt);
        attitude_control->relax_attitude_controllers();
        sub.last_pilot_heading = ahrs.yaw_sensor;

        Ux = 0;
        Uy = 0;
        Uz = 0;

        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);


    /////////////////////////////////////// OPTICAL FLOW CONTROL ////////////////////////////////////////////////////:

    uint32_t tnow = AP_HAL::millis();



    //////////////// Stabilize //////////////

    float target_roll, target_pitch;

    // convert pilot input to lean angles
    // To-Do: convert sub.get_pilot_desired_lean_angles to return angles as floats
    // TODO2: move into mode.h
    sub.get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, sub.aparm.angle_max);

    // get pilot's desired yaw rate
    float yaw_input = channel_yaw->pwm_to_angle_dz_trim(channel_yaw->get_dead_zone() * sub.gain, channel_yaw->get_radio_trim());
    float target_yaw_rate = sub.get_pilot_desired_yaw_rate(yaw_input);

    // call attitude controller
    // update attitude controller targets

    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        sub.last_pilot_heading = ahrs.yaw_sensor;
        sub.last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < sub.last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0;  // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            sub.last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control->input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, sub.last_pilot_heading, true);
        }
    }

    //////////////// Optical Flow ////////////////

    const Vector2f& V = sub.optflow.getState().bodyRate;
    const float Vz = sub.optflow.getState().body_Vz;
    uint8_t quality = sub.optflow.quality();
    uint32_t dt = tnow - previous;

    Vector3f V_robot= {-V.x, V.y, Vz}; //Because the y axis is reversed on a image and because the sensor gives us the speed/displacement of the image relatively to the scene

    previous = tnow;

    PIDx.kP(g.Px);
    PIDx.kI(g.Ix);
    PIDx.kD(g.Dx);

    PIDy.kP(g.Py);
    PIDy.kI(g.Iy);
    PIDy.kD(g.Dy);

    PIDz.kP(g.Pz);
    PIDz.kI(g.Iz);
    PIDz.kD(g.Dz);

    Ux = PIDx.update_all(0, V_robot.x, dt*1e-3);
    Uy = PIDy.update_all(0, V_robot.y, dt*1e-3);
    Uz = PIDz.update_all(0, V_robot.z, dt*1e-3);

    // Preprocess
    Uy = Uy/2 + 0.5; //From -1 <-> 1 to 0 <-> 1

    Ux = std::min(1.0, std::max(-1.0, Ux));
    Uy = std::min(1.0, std::max(0.0, Uy));
    Uz = std::min(1.0, std::max(-1.0, Uz));



    //if (tnow - before >  1000) { //TODO Add mav param for debug
    //       gcs().send_text(MAV_SEVERITY_INFO, "Value (Ux : %.3f, Vx : %.3f)", Ux, V_robot.x);
    //       gcs().send_text(MAV_SEVERITY_INFO, "Value (Uy : %.3f, Vy : %.3f)", Uy, V_robot.y);
    //       gcs().send_text(MAV_SEVERITY_INFO, "Value (Uz : %.3f, Vz : %.3f)", Uz, V_robot.z);
    //       gcs().send_text(MAV_SEVERITY_INFO, "Qual : %d", (int)quality);
    //       before = tnow;
    //}

    motors.set_lateral(Ux);
    motors.set_throttle(Uy); 
    motors.set_forward(Uz);
    //if (quality > 125) { // Don't try to stabalize if you don't have enough sift point
    //	motors.set_lateral(Ux);
    //	motors.set_throttle(Uy);
    //	motors.set_forward(Uz);
    //}
}

