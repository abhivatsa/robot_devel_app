#include <cmath>
#include <unistd.h>
#include <iostream>

#include "MotionPlanning/ForwardKinematics.h"
#include "MotionPlanning/IK6AxisInline.h"
#include "MotionPlanning/Jacobian.h"

enum class DriveState
{
    INITIALIZE,
    NOT_READY_TO_SWITCH_ON,
    SWITCHED_ON,
    SWITCH_TO_OPERATION,
    OPERATION_ENABLED,
    ERROR,
};

enum class OperationModeState
{
    POSITION_MODE = 8,
    VELOCITY_MODE = 9,
    TORQUE_MODE = 10,
};

enum class SafetyStates
{
    INITIALIZE,
    INITIALIZE_DRIVES,
    SAFETY_CHECK,
    READY_FOR_OPERATION,
    OPERATION,
    ERROR,
};

double gear_ratio[6] = {161, 161, 161, 121, 121, 121};
double rated_torque[6] = {2.08, 2.08, 2.08, 0.62, 0.62, 0.62};
double enc_count[6] = {524288, 524288, 524288, 262144, 262144, 262144};
double pos_limit[6] = {0.95*M_PI, M_PI/2, 2*M_PI/3, 0.95*M_PI, 5*M_PI/6, 0.99*M_PI};
double vel_limit[6] = {M_PI/6, M_PI/6, M_PI/6, M_PI/3, M_PI/3, M_PI/3};
double torque_limit[6] = {180, 180, 180, 50, 50, 50};

int conv_to_target_pos(double rad, int jnt_ctr)
{
    // input in radians, output in encoder count (SEE Object 0x607A)
    return (int)(enc_count[jnt_ctr] * gear_ratio[jnt_ctr] * rad / (2 * M_PI)); 
}

double conv_to_actual_pos(int count, int jnt_ctr)
{
    // input in encoder count, Output in radians (SEE Object 0x6064)
    return (count / (enc_count[jnt_ctr] * gear_ratio[jnt_ctr]) * (2 * M_PI)); 
}

int conv_to_target_velocity(double rad_sec, int jnt_ctr)
{
    // input in rad/sec, Output in rpm (SEE Object 0X60FF)
    return (int)(rad_sec / (2 * M_PI) * 60 * gear_ratio[jnt_ctr]);
}

double conv_to_actual_velocity(int rpm, int jnt_ctr)
{
    // input in rpm , Output in rad/sec SEE Object (0x606C)
    return (2 * M_PI * rpm / (60 * gear_ratio[jnt_ctr]));
}

int conv_to_target_torque(double torq_val, int jnt_ctr)
{
    // input is torque in N-m, Output is in per thousand of rated torque (SEE Object 0x6071)
    return (int)(torq_val / (rated_torque[jnt_ctr] * gear_ratio[jnt_ctr]) * 1000);
}

double conv_to_actual_torque(int torq_val, int jnt_ctr)
{
    // input torq in terms of per thousand of rated torque, Output is in N-m (SEE object 0x6077)
    return (torq_val / 1000 * rated_torque[jnt_ctr] * gear_ratio[jnt_ctr]);
}


struct JointData
{

    void setZero()
    {
        for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
        {
            joint_position[jnt_ctr] = 0;
            joint_velocity[jnt_ctr] = 0;
            joint_torque[jnt_ctr] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
        }
    }

    int joint_position[6];
    int joint_velocity[6];
    int joint_torque[6];
    int target_position[6];
    int target_velocity[6];
    int target_torque[6];
};

struct AppData
{
    void setZero()
    {
        switch_to_operation = false;
        initialize_drives = false;
        initialize_system = false;
        trigger_error = false;
        safety_process_status = false;
        drive_initialized = false;
        safety_check_done = false;
        reset_error = false;
        operation_enable_status = false;

        for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
        {
            actual_position[jnt_ctr] = 0;
            actual_velocity[jnt_ctr] = 0;
            actual_torque[jnt_ctr] = 0;
            cart_pos[3] = 0;
            target_position[jnt_ctr] = 0;
            target_velocity[jnt_ctr] = 0;
            target_torque[jnt_ctr] = 0;
            drive_operation_mode = OperationModeState::POSITION_MODE;
            switched_on = false;
            simulation_mode = false;
        }
    }

    double actual_position[6];
    double actual_velocity[6];
    double actual_torque[6];
    double cart_pos[3];
    double target_position[6];
    double target_velocity[6];
    double target_torque[6];
    OperationModeState drive_operation_mode;
    bool switched_on;
    bool simulation_mode;

    bool trigger_error;
    bool safety_process_status;
    bool initialize_system;
    bool initialize_drives;
    bool drive_initialized;
    bool switch_to_operation;
    bool safety_check_done;
    bool operation_enable_status;
    bool reset_error;
};

struct SystemStateData
{
    void setZero()
    {
        current_state = DriveState::INITIALIZE;
        drive_operation_mode = OperationModeState::POSITION_MODE;
        state = SafetyStates::INITIALIZE;
        initialize_drives = false;
        switch_to_operation = false;

        status_switched_on = false;
        status_operation_enabled = false;
        safety_controller_enabled = false;
        trigger_error_mode = false;

        safety_check_done = false;
        start_safety_check = false;
        for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
        {
            drive_enable_for_operation[jnt_ctr] = false;
        }
    }

    DriveState current_state;
    OperationModeState drive_operation_mode;
    SafetyStates state;
    // Variables for Drive status
    bool status_switched_on;
    bool status_operation_enabled;

    // To determine where safety Code started Running
    bool safety_controller_enabled;

    // both Variables ghas to Come from Safety Code
    bool trigger_error_mode; // Chaged from safety controller

    // Variables to initialize and check the status of safety code at 1000Hz
    bool start_safety_check;
    bool safety_check_done;

    bool initialize_drives;
    bool switch_to_operation;

    bool drive_enable_for_operation[6];
};

JointData *joint_data_ptr;
SystemStateData *system_state_data_ptr;
AppData *app_data_ptr;


void read_data();
void write_data();
bool check_limits();

int joint_pos_limit_check();
int joint_vel_limit_check();
int joint_torq_limit_check();

