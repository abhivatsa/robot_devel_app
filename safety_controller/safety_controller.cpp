#include "safety_controller.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include <bits/stdc++.h>
#include <sys/time.h>

int main()
{
    /* the size (in bytes) of shared memory object */
    const int SIZE_JointData = sizeof(JointData);
    const int SIZE_SystemStateData = sizeof(SystemStateData);
    const int SIZE_AppData = sizeof(AppData);

    int shm_fd_jointData;
    int shm_fd_systemStateData;
    int shm_fd_appData;

    /* open the shared memory object */
    shm_fd_jointData = shm_open("JointData", O_CREAT | O_RDWR, 0666);
    shm_fd_systemStateData = shm_open("SystemStateData", O_CREAT | O_RDWR, 0666);
    shm_fd_appData = shm_open("AppData", O_CREAT | O_RDWR, 0666);

    ftruncate(shm_fd_jointData, SIZE_JointData);
    ftruncate(shm_fd_systemStateData, SIZE_SystemStateData);
    ftruncate(shm_fd_appData, SIZE_AppData);

    joint_data_ptr = static_cast<JointData *>(mmap(0, SIZE_JointData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_jointData, 0));
    system_state_data_ptr = static_cast<SystemStateData *>(mmap(0, SIZE_SystemStateData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_systemStateData, 0));
    app_data_ptr = static_cast<AppData *>(mmap(0, SIZE_AppData, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_appData, 0));

    joint_data_ptr->setZero();
    system_state_data_ptr->setZero();

    bool run_safety_loop = true;

    while (run_safety_loop)
    {
        switch (system_state_data_ptr->state)
        {
        case SafetyStates::INITIALIZE:
        { /* code */
            system_state_data_ptr->safety_controller_enabled = true;
            if (system_state_data_ptr->current_state == DriveState::NOT_READY_TO_SWITCH_ON)
            {
                // send signal to motion planner
                if (app_data_ptr->initialize_system) // motion planner initialized?
                {
                    app_data_ptr->safety_process_status = true;
                    if (app_data_ptr->initialize_drives) // initialize drive? from  motion planner
                    {
                        system_state_data_ptr->state = SafetyStates::INITIALIZE_DRIVES;
                    }
                }
            }
            break;
        }
        case SafetyStates::INITIALIZE_DRIVES:
        {
            system_state_data_ptr->initialize_drives = true;
            if (system_state_data_ptr->current_state == DriveState::SWITCHED_ON)
            {
                app_data_ptr->trigger_error = false;
                app_data_ptr->drive_initialized = true;
                if (system_state_data_ptr->start_safety_check)
                {
                    system_state_data_ptr->state = SafetyStates::SAFETY_CHECK;
                }
            }

            if (system_state_data_ptr->current_state == DriveState::ERROR)
            {
                app_data_ptr->trigger_error = true;
                app_data_ptr->drive_initialized = true;
                // do someting
            }
            break;
        }
        case SafetyStates::SAFETY_CHECK:
        {
            if (check_limits())
            {
                system_state_data_ptr->state = SafetyStates::READY_FOR_OPERATION;
                app_data_ptr->trigger_error = false;
            }
            else
            {
                // safet check failed
                app_data_ptr->trigger_error = true;
                run_safety_loop = false;
            }
            app_data_ptr->safety_check_done = true;
            system_state_data_ptr->safety_check_done = true;

            break;
        }
        case SafetyStates::READY_FOR_OPERATION:
        {

            read_data();

            if (check_limits())
            {

                for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
                {
                    app_data_ptr->target_position[jnt_ctr] = joint_data_ptr->joint_position[jnt_ctr];
                    app_data_ptr->target_torque[jnt_ctr] = joint_data_ptr->joint_torque[jnt_ctr];
                }

                if (app_data_ptr->switch_to_operation) // switch to operation? from motion planner
                {
                    system_state_data_ptr->switch_to_operation = true;
                    if (system_state_data_ptr->current_state == DriveState::OPERATION_ENABLED)
                    {
                        system_state_data_ptr->state = SafetyStates::OPERATION;
                    }
                }
            }
            else
            {

                app_data_ptr->trigger_error = true;
                run_safety_loop = false;
            }

            break;
        }
        case SafetyStates::OPERATION:
        {
            if (system_state_data_ptr->current_state == DriveState::OPERATION_ENABLED)
            {
                app_data_ptr->operation_enable_status = true;
                // read write
                check_limits();
                read_data();
                write_data();
            }
            else if (system_state_data_ptr->current_state == DriveState::ERROR)
            {
                app_data_ptr->trigger_error = true;
                system_state_data_ptr->state = SafetyStates::ERROR;
                // send signal to motion planner
            }
            break;
        }
        case SafetyStates::ERROR:
        {
            app_data_ptr->setZero();
            if (system_state_data_ptr->current_state == DriveState::SWITCHED_ON)
            {
                system_state_data_ptr->state = SafetyStates::READY_FOR_OPERATION;
            }
            break;
        }
        default:
            break;
        }
        usleep(1000);
    }
}

void read_data()
{

    for (unsigned int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
    {
        app_data_ptr->actual_position[jnt_ctr] = conv_to_actual_pos(joint_data_ptr->joint_position[jnt_ctr], jnt_ctr);
        app_data_ptr->actual_velocity[jnt_ctr] = conv_to_actual_velocity(joint_data_ptr->joint_velocity[jnt_ctr], jnt_ctr);
        app_data_ptr->actual_torque[jnt_ctr] = conv_to_actual_torque(joint_data_ptr->joint_torque[jnt_ctr], jnt_ctr);
    }
}

void write_data()
{

    if (!system_state_data_ptr->trigger_error_mode && system_state_data_ptr->status_operation_enabled)
    {
        system_state_data_ptr->drive_operation_mode = app_data_ptr->drive_operation_mode;
        for (unsigned int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
        {
            joint_data_ptr->target_position[jnt_ctr] = conv_to_target_pos(app_data_ptr->target_position[jnt_ctr], jnt_ctr);
            joint_data_ptr->target_velocity[jnt_ctr] = conv_to_target_velocity(app_data_ptr->target_velocity[jnt_ctr], jnt_ctr);
            joint_data_ptr->target_torque[jnt_ctr] = conv_to_target_torque(app_data_ptr->target_torque[jnt_ctr]);
        }
    }
}

bool check_limits()
{

    system_state_data_ptr->trigger_error_mode = joint_pos_limit_check() &&
                                                        joint_vel_limit_check() &&
                                                        joint_torq_limit_check()
                                                    ? false
                                                    : true;

    return !system_state_data_ptr->trigger_error_mode;
}

int joint_pos_limit_check()
{

    for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
    {
        if ((conv_to_actual_pos(joint_data_ptr->joint_position[jnt_ctr], jnt_ctr)) > pos_limit[jnt_ctr])
        {
            return 0;
        }
    }

    return 1;
}

int joint_vel_limit_check()
{

    for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
    {
        if ((conv_to_actual_velocity(joint_data_ptr->joint_velocity[jnt_ctr], jnt_ctr)) > vel_limit[jnt_ctr])
        {
            return 0;
        }
    }

    return 1;
}

int joint_torq_limit_check()
{

    for (int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
    {
        if ((conv_to_actual_torque(joint_data_ptr->joint_torque[jnt_ctr], jnt_ctr)) > torque_limit[jnt_ctr])
        {
            return 0;
        }
    }

    return 1;
}
