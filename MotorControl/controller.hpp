#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

class OpticalEncoder;
class Motor;

typedef struct{
    float vel_limit = 20000.0f;  // [degree/s]
    float accel_limit = 5000.0f; // [degree/s^2]
    float decel_limit = 5000.0f; // [degree/s^2]
    float A_per_css = 0.0f;      // [A/(degree/s^2)]
}trap_config_t;

class TrapezoidalTrajectory {
public:

    struct Step_t {
        float Y;
        float Yd;
        float Ydd;
    };

    TrapezoidalTrajectory(trap_config_t& config);
    bool planTrapezoidal(float Xf, float Xi, float Vi,
                         float Vmax, float Amax, float Dmax);
    Step_t eval(float t);

    //Axis* axis_ = nullptr;  // set by Axis constructor
    trap_config_t& config_;

    float Xi_ = 0.0f;
    float Xf_ = 0.0f;
    float Vi_ = 0.0f;

    float Ar_ = 0.0f;
    float Vr_ = 0.0f;
    float Dr_ = 0.0f;

    float Ta_ = 0.0f;
    float Tv_ = 0.0f;
    float Td_ = 0.0f;
    float Tf_ = 0.0f;

    float yAccel_ = 0.0f;
    float t_ = 0.0f;
};

typedef struct{
    //ControlMode_t control_mode = CTRL_MODE_POSITION_CONTROL;  //see: Motor_control_mode_t
    float pos_gain = 20.0f;  // [(degree/s) / degree]
    float vel_gain = 5.0f / 10000.0f;  // [A/(degree/s)]
    // float vel_gain = 5.0f / 200.0f, // [A/(rad/s)] <sensorless example>
    float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(degree/s * s)]
    float vel_limit = 20000.0f;        // [degree/s]
    float acc_limit = 20000.0f;        // [degree/s^2]
    float jerk_limit = 20000.0f;        // [degree/s^3]
    float vel_limit_tolerance = 1.2f;  // ratio to vel_lim. 0.0f to disable
    float vel_ramp_rate = 10000.0f;  // [(degree/s) / s]
}ctrl_config_t;

class Controller {
public:
    enum Error_t {
        ERROR_NONE = 0,
        ERROR_OVERSPEED = 0x01,
    };

    // Note: these should be sorted from lowest level of control to
    // highest level of control, to allow "<" style comparisons.
    enum ControlMode_t{
        CTRL_MODE_VOLTAGE_CONTROL = 0,
        CTRL_MODE_CURRENT_CONTROL = 1,
        CTRL_MODE_VELOCITY_CONTROL = 2,
        CTRL_MODE_POSITION_CONTROL = 3,
        CTRL_MODE_TRAJECTORY_CONTROL = 4
    };


    Controller(ctrl_config_t& config, trap_config_t& trap_conf);
    void reset();
    void set_error(Error_t error);

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);

    // Trajectory-Planned control
    void move_to_pos(float goal_point, float cur_pos);
    
    // TODO: make this more similar to other calibration loops
    bool update(OpticalEncoder& encoder_, float pos_feedbak, float& ret_Iq, float Iq_limit);
    //bool update(float pos_estimate, float vel_estimate, float* current_setpoint);

    ctrl_config_t& config_;
    ControlMode_t mode_ = CTRL_MODE_POSITION_CONTROL;
    //Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: anticogging overhaul:
    // - expose selected (all?) variables on protocol
    // - make calibration user experience similar to motor & encoder calibration
    // - use python tools to Fourier transform and write back the smoothed map or Fourier coefficients
    // - make the calibration persistent

    typedef struct {
        int index;
        float *cogging_map;
        bool use_anticogging;
        bool calib_anticogging;
        float calib_pos_threshold;
        float calib_vel_threshold;
    } Anticogging_t;
    Anticogging_t anticogging_ = {
        .index = 0,
        .cogging_map = nullptr,
        .use_anticogging = false,
        .calib_anticogging = false,
        .calib_pos_threshold = 1.0f,
        .calib_vel_threshold = 1.0f,
    };

    Error_t error_ = ERROR_NONE;
    // variables exposed on protocol
    float pos_err_ = 0.0f;          // [degree]
    float pos_abs_err_ = 0.0f;
    float vel_err_ = 0.0f;          // [degree]
    float vel_abs_err_ = 0.0f;
    float pos_setpoint_ = 0.0f;     // [degree]
    float vel_setpoint_ = 0.0f;     // [degree / s]
    float vel_integrator_current_ = 0.0f;  // [A]
    float current_setpoint_ = 0.0f;        // [A]
    float vel_ramp_target_ = 0.0f;
    bool  vel_ramp_enable_ = true;
    float last_vel_set = 0.0f;
    float last_acc_set = 0.0f;
    float last_jerk_set = 0.0f;
    
    TrapezoidalTrajectory trap_;
    float pos_estimate = 0.0f;
    float vel_estimate = 0.0f;
};
#endif // __CONTROLLER_HPP
