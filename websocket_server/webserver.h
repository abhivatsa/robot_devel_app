#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/dispatch.hpp>
#include <boost/asio/strand.hpp>
#include <algorithm>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <cmath>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <unistd.h>
#include "rapidjson/document.h" // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h"
#include <fstream>
#include <rapidjson/istreamwrapper.h>

// structer for system data
enum SystemState
{
    POWER_OFF,
    INITIALIZING_SYSTEM,
    HARWARE_CHECK,
    READY,
    IN_EXECUTION,
    ERROR
};

enum CommandType{
    NONE,
    JOG,
    HAND_CONTROL,
    GRAVITY,
    MOVE_TO
};

// System data (State)
struct SystemData
{
    SystemState getSystemState() const { return system_state; }
    void setSystemState(SystemState state) { system_state = state; }
    void powerOn() { request = system_state == SystemState::POWER_OFF ? 1 : 0; }
    void powerOff() { request = system_state == SystemState::READY ? -1 : 0; }
    int request = 0;

private:
    SystemState system_state = SystemState::POWER_OFF;
};

// Robot state
struct RobotState
{
    void setZero()
    {
        for (int i = 0; i < 6; i++)
        {
            joint_position[i] = 0;
            cart_position[i] = 0;
            joint_velocity[i] = 0;
            joint_torque[i] = 0;
        }
    }
    double cart_position[6];
    double joint_position[6];
    double joint_velocity[6];
    double joint_torque[6];
};

// Command data
struct CommandData
{
    void setJog(int index, int dir, int mode)
    {
        this->type = CommandType::JOG;
        jog_data.index = index - 1;
        jog_data.dir = dir;
        jog_data.type = mode;
    }
    void setHandControl(){
        this->type = CommandType::HAND_CONTROL;
    }
    void setGravity(){
        this->type = CommandType::GRAVITY;
    }
    void setMoveTo(double goal[6], int type)
    {
        for (int i = 0; i < 6; i++)
        {
            move_to_data.goal_position[i] = goal[i];
        }
        move_to_data.type = type;
        this->type = CommandType::MOVE_TO;
    }
    CommandType type;
    struct
    {
        int index;
        int dir;
        int type;
    } jog_data;
    struct
    {
        int type;
        double goal_position[6];
    } move_to_data;
};