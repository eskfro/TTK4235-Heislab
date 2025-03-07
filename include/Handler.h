#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>
#include <assert.h>

#include "Elevator.h"
#include "OrderQueue.h"
#include "Matrix.h"

#define DOOR_OPEN_TIME_MS 1000
#define DOOR_OBS_TIME_MS 1000
#define LOOP_DELAY_MS 20

typedef struct {
    bool go;
} Handler;


void handler_run_matrix();
void handler_updateMatrix(Matrix* m);
void handler_resetLamps();
void handler_updateQueue(Matrix* p_m, OrderQueue* p_q, Elevator* p_e);
void handler_printArrived(int floor);
void handler_printElevatorStates(Elevator* e);


//-----------------------------------FROM FIRST TRY-----------------
void handler_run_v1();
void sortQueue(OrderQueue* p_queue, Elevator* p_elevator);
void processRequests(OrderQueue* p_queue, Elevator* p_elevator);






