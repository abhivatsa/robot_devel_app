#ifndef MOVE_TO_H
#define MOVE_TO_H

#include "pt_to_pt_planner.h"
#include "robot_motion_planner.h"

double move_to()
{

    double ini_pos[6];
    double final_pos[6] = {0, 0, -1.57, 0, -1.57, 0};

    for (unsigned int jnt_ctr = 0; jnt_ctr < 6; jnt_ctr++)
    {
        ini_pos[jnt_ctr] = app_data_ptr->actual_position[jnt_ctr];
    }
    
    pt_to_pt_mvmt(ini_pos, final_pos);
    
    return 0;
}

#endif