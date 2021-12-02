#include "motor.hpp"
#include "optical_encoder.hpp"
#include "controller.hpp"
#include "utils.hpp"
#include <math.h>

// A sign function where input 0 has positive sign (not 0)
inline float sign_hard(float val)
{
	if(val > 0.0f) return 1.0f;
	return -1.0f;
}

// Symbol                     Description
// Ta, Tv and Td              Duration of the stages of the AL profile
// Xi and Vi                  Adapted initial conditions for the AL profile
// Xf                         Position set-point
// s                          Direction (sign) of the trajectory
// Vmax, Amax, Dmax and jmax  Kinematic bounds
// Ar, Dr and Vr              Reached values of acceleration and velocity

TrapezoidalTrajectory::TrapezoidalTrajectory(trap_config_t& config) : config_(config) {}

bool TrapezoidalTrajectory::planTrapezoidal(float Xf, float Xi, float Vi,
                                            float Vmax, float Amax, float Dmax) {
    float dX = Xf - Xi;  // Distance to travel
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance
	float dXstop = 0.0f; // Minimum stopping displacement
	if(Vi > 0.0f)
	{
		dXstop = stop_dist;
	}
	else
	{
		dXstop = -stop_dist;
	}
    float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any)
    Ar_ = s * Amax;  // Maximum Acceleration (signed)
    Dr_ = -s * Dmax; // Maximum Deceleration (signed)
    Vr_ = s * Vmax;  // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel
    // aka "double deceleration move" in the paper
    if ((s * Vi) > (s * Vr_)) {
        Ar_ = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed)
    Ta_ = (Vr_ - Vi) / Ar_;
    Td_ = -Vr_ / Dr_;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed
    float dXmin = 0.5f*Ta_*(Vr_ + Vi) + 0.5f*Td_*Vr_;

    // Are we displacing enough to reach cruising speed?
    if (s*dX < s*dXmin) {
        // Short move (triangle profile)
        Vr_ = s * sqrt((Dr_*SQ(Vi) + 2*Ar_*Dr_*dX) / (Dr_ - Ar_));
        Ta_ = fmax(0.0f, (Vr_ - Vi) / Ar_);
        Td_ = fmax(0.0f, -Vr_ / Dr_);
        Tv_ = 0.0f;
    } else {
        // Long move (trapezoidal profile)
        Tv_ = (dX - dXmin) / Vr_;
    }

    // Fill in the rest of the values used at evaluation-time
    Tf_ = Ta_ + Tv_ + Td_;
    Xi_ = Xi;
    Xf_ = Xf;
    Vi_ = Vi;
    yAccel_ = Xi + Vi*Ta_ + 0.5f*Ar_*SQ(Ta_); // pos at end of accel phase

    return true;
}

TrapezoidalTrajectory::Step_t TrapezoidalTrajectory::eval(float t) {
    Step_t trajStep;
    if (t < 0.0f) {  // Initial Condition
        trajStep.Y   = Xi_;
        trajStep.Yd  = Vi_;
        trajStep.Ydd = 0.0f;
    } else if (t < Ta_) {  // Accelerating
        trajStep.Y   = Xi_ + Vi_*t + 0.5f*Ar_*SQ(t);
        trajStep.Yd  = Vi_ + Ar_*t;
        trajStep.Ydd = Ar_;
    } else if (t < Ta_ + Tv_) {  // Coasting
        trajStep.Y   = yAccel_ + Vr_*(t - Ta_);
        trajStep.Yd  = Vr_;
        trajStep.Ydd = 0.0f;
    } else if (t < Tf_) {  // Deceleration
        float td     = t - Tf_;
        trajStep.Y   = Xf_ + 0.5f*Dr_*SQ(td);
        trajStep.Yd  = Dr_*td;
        trajStep.Ydd = Dr_;
    } else if (t >= Tf_) {  // Final Condition
        trajStep.Y   = Xf_;
        trajStep.Yd  = 0.0f;
        trajStep.Ydd = 0.0f;
    } else {
        // TODO: report error here
    }

    return trajStep;
}

Controller::Controller(ctrl_config_t& config, trap_config_t& trap_conf) :
    config_(config),
    trap_(trap_conf)
{}

void Controller::reset() {
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

void Controller::set_error(Error_t error) {
    error_ = error;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    mode_ = CTRL_MODE_POSITION_CONTROL;
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    mode_ = CTRL_MODE_VELOCITY_CONTROL;
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    mode_ = CTRL_MODE_CURRENT_CONTROL;
}

void Controller::move_to_pos(float goal_point, float cur_pos) {
    
   trap_.planTrapezoidal(goal_point, cur_pos, vel_estimate,
                                 trap_.config_.vel_limit,
                                 trap_.config_.accel_limit,
                                 trap_.config_.decel_limit);
    trap_.t_ = 0.0f;
    mode_ = CTRL_MODE_TRAJECTORY_CONTROL;
}

bool Controller::update(OpticalEncoder& encoder_, float pos_feedbak, float& ret_Iq, float Iq_limit){
    const float current_meas_period = 0.0000625f;
    pos_estimate = (encoder_.count_in_cpr_ / (float)(encoder_.config_.cpr) + (float)encoder_.turn_) * 360.0f;
    vel_estimate = (encoder_.vel_estimate_ / (float)(encoder_.config_.cpr)) * 360.0f;
    // Only runs if anticogging_.calib_anticogging is true; non-blocking

    // Trajectory control
    if (mode_ == CTRL_MODE_TRAJECTORY_CONTROL) {
        // Note: uint32_t loop count delta is OK across overflow
        // Beware of negative deltas, as they will not be well behaved due to uint!
        if (trap_.t_ > trap_.Tf_) {
            // Drop into position control mode when done to avoid problems on loop counter delta overflow
            mode_ = CTRL_MODE_POSITION_CONTROL;
            // pos_setpoint already set by trajectory
            vel_setpoint_ = 0.0f;
            current_setpoint_ = 0.0f;
        } else {
            TrapezoidalTrajectory::Step_t traj_step =trap_.eval(trap_.t_);
            pos_setpoint_ = traj_step.Y;
            vel_setpoint_ = traj_step.Yd;
            current_setpoint_ = 0.0f;
        }
        trap_.t_ += current_meas_period;     
    }

    // Ramp rate limited velocity setpoint
    if (mode_ == CTRL_MODE_VELOCITY_CONTROL && vel_ramp_enable_) {
        float max_step_size = 0.0f;
        float full_step = vel_ramp_target_ - vel_setpoint_;

        if(full_step * vel_setpoint_ > 0)
        {
           max_step_size = current_meas_period * config_.vel_ramp_rate;
        }      
        else
        {
            max_step_size = current_meas_period * config_.vel_ramp_rate * 5.0f;
        }
            
        
        float step;
        if (fabsf(full_step) > max_step_size) {
            step = std::copysignf(max_step_size, full_step);
        } else {
            step = full_step;
        }
        vel_setpoint_ += step;
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float vel_des = vel_setpoint_;
    if (mode_ >= CTRL_MODE_POSITION_CONTROL) {
        float pos_err;
        pos_err = pos_setpoint_ - pos_feedbak;
        pos_err_ = pos_err;
        pos_abs_err_ = fabsf(pos_err);
        vel_des += config_.pos_gain * pos_err;
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    
    if (vel_des > vel_lim) vel_des = vel_lim;
    if (vel_des < -vel_lim) vel_des = -vel_lim;
   
    // Velocity control
    float Iq = current_setpoint_;

    float v_err = vel_des - vel_estimate;
    vel_err_ = v_err;
    vel_abs_err_ = fabsf(v_err);
    if (mode_ >= CTRL_MODE_VELOCITY_CONTROL) {
        Iq += config_.vel_gain * v_err;
        // Velocity integral action before limiting
        Iq += vel_integrator_current_;
    }
    
    // Current limiting
    float Ilim = Iq_limit;
    bool limited = false;
    if (Iq > Ilim) {
        limited = true;
        Iq = Ilim;
    }
    if (Iq < -Ilim) {
        limited = true;
        Iq = -Ilim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (mode_ < CTRL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_current_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            if(fabsf(vel_integrator_current_) > 0.0001f)
                vel_integrator_current_ *= 0.99f;
        } else {
            vel_integrator_current_ += (config_.vel_integrator_gain * current_meas_period) * v_err;
        }
    }

    ret_Iq = Iq;
    return true;
}
