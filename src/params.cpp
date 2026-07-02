#include "params.h"

MCStatus_t MController::init(void) {
    return STATUS_OK;
}

MCStatus_t MController::update(void) {
    this->cmd_vel.x     = mros_get_cmd_vel_lin_x();
    this->cmd_vel.y     = mros_get_cmd_vel_lin_y();
    this->cmd_vel.theta = mros_get_cmd_vel_ang_z();

    return STATUS_OK;
}