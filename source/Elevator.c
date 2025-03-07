#include <time.h>
#include "../include/Elevator.h"

void elev_stop(Elevator *p_elevator)
{
    elev_setMotorDir(p_elevator, DIRN_STOP);
    elevio_stopLamp(1);
}

Elevator elev_createElevator()
{   
    Elevator elev = {1, 1, 0, 0, 0, 0, DIRN_STOP};
    return elev;
}

void elev_initElevator(Elevator* p_elevator)
{
    while (elevio_floorSensor() == -1)  
    {
        elev_setMotorDir(p_elevator, DIRN_UP);
    }
    elev_setMotorDir(p_elevator, DIRN_STOP);

    p_elevator->current_floor = elevio_floorSensor();
    elevio_floorIndicator(p_elevator->current_floor);
    elevio_doorOpenLamp(p_elevator->door_open);

    //Set previous floor
    if (p_elevator->current_floor == 0) p_elevator->previous_floor = 1;
    else if (p_elevator->current_floor == 3) p_elevator->previous_floor = 2;
    else p_elevator->previous_floor = (p_elevator->current_floor - 1);
    printf("ELEVATOR -> INIT : cf, pf = {%i, %i}\n", p_elevator->current_floor, p_elevator->previous_floor);
    return;
}

void elev_moveTo(Elevator *p_elevator, int floor)
{   
    if (floor == 0) p_elevator->direction = 1;
    if (floor == 3) p_elevator->direction = 0;

    //Floor lights
    int floor_sensor = elevio_floorSensor();

    if (floor_sensor != -1) 
    {
        if (floor_sensor != p_elevator->current_floor) p_elevator->current_floor = floor_sensor;
        elevio_floorIndicator(floor_sensor);  
    }

    //Arrived
    if (floor_sensor == floor) 
    {
        //Update floors
        p_elevator->previous_floor = p_elevator->current_floor;
        p_elevator->current_floor = floor;
        elev_setMotorDir(p_elevator, DIRN_STOP);
        p_elevator->has_arrived = true;
        return;
    }

    //UP and DOWN
    if (p_elevator->current_floor > floor)
    {
        elev_setMotorDir(p_elevator, DIRN_DOWN);
        p_elevator->direction = 1;
    } 
    else if (p_elevator->current_floor < floor)
    {
        elev_setMotorDir(p_elevator, DIRN_UP);
        p_elevator->direction = 0;
    }
}



void elev_openDoor(Elevator *p_elevator)
{
    if (p_elevator->motor_dir == DIRN_STOP) 
    {
        elevio_doorOpenLamp(1);
        p_elevator->door_open = true;
    }
    printf("\t\t\t\tdoor OPEN\n\n");
}



void elev_closeDoor(Elevator *p_elevator)
{   
    elevio_doorOpenLamp(0);
    p_elevator->door_open = false;
    printf("\t\t\t\tdoor CLOSED\n\n");
}


void elev_updateCurrentFloor(Elevator *p_elevator)
{
    p_elevator->current_floor = elevio_floorSensor();
}



void elev_setMotorDir(Elevator *p_elevator, MotorDirection dir)
{
    elevio_motorDirection(dir);
    p_elevator->motor_dir = dir;
}



void elev_emergencyStop(Elevator *p_elevator)
{
    elevio_motorDirection(DIRN_STOP);
    p_elevator->motor_dir = DIRN_STOP;
}





