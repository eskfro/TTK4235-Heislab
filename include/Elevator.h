#pragma once

#include <stdbool.h>
#include <unistd.h>
#include "OrderQueue.h"
#include "../source/driver/elevio.h"

typedef struct {
    int current_floor;
    int previous_floor;
    int direction;
    bool door_open;
    bool has_arrived;
    bool is_obs;
    MotorDirection motor_dir;
} Elevator;

Elevator elev_createElevator();
void elev_initElevator(Elevator* p_elevator);
void elev_stop(Elevator* p_elevator);
void elev_moveTo(Elevator* p_elevator, int floor);
void elev_openDoor(Elevator* p_elevator);
void elev_closeDoor(Elevator* p_elevator);
void elev_updateCurrentFloor(Elevator* p_elevator);
void elev_setMotorDir(Elevator* p_elevator, MotorDirection dir);
void elev_emergencyStop(Elevator* p_elevator);

