#include "../include/Handler.h"



void handler_updateQueue(Matrix* p_m, OrderQueue* p_q, Elevator* p_e)
{
    if (p_e->current_floor == N_FLOORS - 1) p_e->direction = 1;
    if (p_e->current_floor == 0) p_e->direction = 0;

    // Check if there's a request at the current floor AND the elevator is stopped
    if (p_e->motor_dir == DIRN_STOP && 
        (p_m->list[p_e->current_floor].cab == 1 || 
        p_m->list[p_e->current_floor].hall_up == 1 || 
        p_m->list[p_e->current_floor].hall_down == 1)) 
    {
        QueueEntry e = {p_e->current_floor, BUTTON_CAB};
        p_q->queue[0] = e;
        return;
    }

    if (matrix_isEmpty(p_m)) {
        p_e->direction = 2; 
        return;
    };

    if (p_e->direction == 0) //UP *************************************** 0 : DIR UP ***********************
    {   
        //First priority loop
        int f = p_e->current_floor + 1;
        while (f < N_FLOORS) {
            if (p_m->list[f].cab == 1 || p_m->list[f].hall_up == 1 ) {
                p_q->queue[0] = (QueueEntry){f, BUTTON_CAB};
                return;
            }
            f = f + 1;
        }

        //Second priority loop
        f = N_FLOORS - 1;
        while (f > p_e->current_floor){
            if (p_m->list[f].hall_down == 1) {
                p_q->queue[0] = (QueueEntry){f, BUTTON_CAB};
                return;
            }
            f = f - 1;
        }
        //SET DIRECTION X IF NO ENTRIES WERE FOUND
        p_e->direction = 2;
    } 
    else if (p_e->direction == 1) //DOWN ******************************** 1 : DIR DOWN *************************
    {
        //First priority loop
        int f = p_e->current_floor - 1;
        while (f >= 0){
            if (p_m->list[f].cab == 1 || p_m->list[f].hall_down == 1) {
                p_q->queue[0] = (QueueEntry){f, BUTTON_CAB};
                return; 
            }
            f = f - 1;
        }

        //Second priority loop
        f = 0;
        while (f < p_e->current_floor){
            if (p_m->list[f].hall_up == 1) {
                p_q->queue[0] = (QueueEntry){f, BUTTON_CAB};
                return;
            }
            f = f + 1;
        }

        //SET DIRECTION X IF NO ENTRIES WERE FOUND
        p_e->direction = 2;
    }
    else //TO DETERMINE WANTED DIR ********************************** 2 : DIR UNKNOWN *********************
    {
        //Unidirectional scan for order
        int floor = p_e->current_floor;
        for (int offset = 1; offset < N_FLOORS; offset++) {
            //Up case
            if (((floor + offset) < N_FLOORS) && matrix_isCallFromFloor(p_m, floor + offset)) {
                p_e->direction = 0;
                return;
            }
            if (((floor - offset) >= 0) && matrix_isCallFromFloor(p_m, floor - offset)) {
                p_e->direction = 1;
                return;
            }
        }
    }
}

void handler_run_matrix()
{
    elevio_init();
    elevio_motorDirection(DIRN_STOP);
    Handler handler = {true};
    OrderQueue q = order_createQueue();
    Elevator elevator = elev_createElevator();
    Matrix m = matrix_createMatrix();
    elev_initElevator(&elevator);
    handler_resetLamps();

    int timer = 0;
    while (handler.go)
    {
        if(elevio_stopButton())
        {
            elev_emergencyStop(&elevator);
            break;
        }
        
        handler_updateMatrix(&m);
        handler_updateQueue(&m, &q, &elevator);
        handler_printElevatorStates(&elevator);
        matrix_printMatrix(&m);
        order_printQueue(&q);

        //Dersom ikke tom kø
        if ((q.queue[0].floor != -1) && (timer <= 0))
        {
            elev_moveTo(&elevator, q.queue[0].floor);
        } 
        else if (elevator.motor_dir != DIRN_STOP)
        {
            elev_setMotorDir(&elevator, DIRN_STOP);  
        }
        
        //Elevator arrived
        if (elevator.has_arrived)
        {   
            handler_printArrived(elevator.current_floor);
            order_removeFromQueue(&q, elevator.current_floor);
            matrix_clearFloor(&m, elevator.current_floor);
            elev_openDoor(&elevator);
            timer += DOOR_OPEN_TIME_MS;
            elevator.has_arrived = false;
        } 
        
        //Door is ready to be closed
        if (timer <= 0 && elevator.door_open == true) 
        {
            if (elevio_obstruction()) 
            {
                timer += DOOR_OBS_TIME_MS;
                printf("\t\t\t\tdoor OBS!\n\n");
            }
            else
            {
                elev_closeDoor(&elevator);
                timer = 0;
            } 
        }

        //Decrement timer
        if (timer > 0) timer -= LOOP_DELAY_MS;
        nanosleep(&(struct timespec){0, LOOP_DELAY_MS*1000*1000}, NULL);        
    }
}


void handler_updateMatrix(Matrix* m) 
{
    for (int etasje = 0; etasje < N_FLOORS; etasje++) {
        for (int button = 0; button < N_BUTTONS; button++) {
            if (elevio_callButton(etasje, (ButtonType)button))
            {
                elevio_buttonLamp(etasje, (ButtonType)button, 1);
                //Case for 0, 1, and 2
                if (button == 0) m->list[etasje].hall_up = 1;
                else if (button == 1) m->list[etasje].hall_down = 1;
                else m->list[etasje].cab = 1;
            }
        }
    }
}


void handler_resetLamps()
{
    for (int etasje = 0; etasje < N_FLOORS; etasje++){
        for (ButtonType buttontype = 0; buttontype < N_BUTTONS; buttontype++) {
            elevio_buttonLamp(etasje, buttontype, 0);       
        }
    }
    elevio_doorOpenLamp(0);
    elevio_stopLamp(0);
}

void handler_printArrived(int floor)
{
    printf("                                                                    _____\n");
    printf("##########################################       arrived @ floor    | %i |\n", floor);
    printf("                                                                    ¨¨¨¨¨\n");
}

void handler_printElevatorStates(Elevator* e)
{
    assert(e->direction >= 0 && e->direction <= 2);
    static int state;
    
    if (e->direction != state){
        if (e->direction == 0){
            printf("direction: UP\n");
        } else if (e->direction == 1){
            printf("direction: DOWN\n");
        } else{
            printf("direction: IDLE\n");
        }
        state = e->direction;
    }
}

//OLD
void sortQueue(OrderQueue* p_queue, Elevator* p_elevator)
{
    for (int i = 0; i < QUEUESIZE; i++)
    {
        for (int j = 0; j < QUEUESIZE - i - 1; j++)
        {
            bool right_direction = p_elevator->motor_dir == DIRN_UP && p_queue->queue[j+1].dir == BUTTON_HALL_UP;
            right_direction = right_direction || (p_elevator->motor_dir == DIRN_DOWN && p_queue->queue[j+1].dir == BUTTON_HALL_DOWN);
            right_direction = right_direction || (p_queue->queue[j+1].dir == BUTTON_CAB);
            
            bool swap = false;
            if (p_elevator->current_floor == 3) {
                swap = p_queue->queue[j].floor < p_queue->queue[j + 1].floor;
            }
            if (p_elevator->current_floor == 0) {
                swap = p_queue->queue[j].floor > p_queue->queue[j + 1].floor;
            }
            if (p_elevator->current_floor == 1) {
                if (p_elevator->motor_dir == DIRN_UP) {
                    if (p_queue->queue[j+1].floor == 2) {
                        swap = true;
                    }
                    else if (p_queue->queue[j+1].floor == 3 && p_queue->queue[j+1].floor == 0) {
                        swap = true;
                    }
                    else {
                        swap = p_queue->queue[j].floor < p_queue->queue[j + 1].floor;
                        
                    }
                } else if (p_elevator->motor_dir == DIRN_DOWN) {
                    if (p_queue->queue[j+1].floor == 0) {
                        swap = true;
                    }
                    else if (p_queue->queue[j+1].floor == 3 && p_queue->queue[j+1].floor == 2) {
                        swap = true;
                    }
                }
            }
            else if (p_elevator->current_floor == 2) {
                if (p_elevator->motor_dir == DIRN_UP) {
                    if (p_queue->queue[j+1].floor == 3) {
                        swap = true;
                    } else if (p_queue->queue[j].floor == 0 && p_queue->queue[j+1].floor == 1) {
                        swap = true;
                    }
                    
                } else if (p_elevator->motor_dir == DIRN_DOWN) {
                    if (p_queue->queue[j+1].floor == 1) {
                        swap = true;
                    } else if (p_queue->queue[j].floor == 3 && p_queue->queue[j+1].floor == 0) {
                        swap = true;
                    }
                }
            }
            bool valid_index = p_queue->queue[j + 1].floor != -1;

            if (swap && right_direction && valid_index) {
                int temp = p_queue->queue[j].floor;
                p_queue->queue[j] = p_queue->queue[j + 1];
                p_queue->queue[j + 1].floor = temp;
            }
        
        }
        
    }
}

//OLD
void processRequests(OrderQueue* p_queue, Elevator* p_elevator)
{
    // sortQueue(p_queue, p_elevator);
    for (int etasje = 0; etasje < N_FLOORS; etasje++){
        for (ButtonType buttontype = 0; buttontype < N_BUTTONS; buttontype++) {
            
            if (elevio_callButton(etasje, buttontype))
            {
                int exist = 0;
                for (int i = 0; i < QUEUESIZE; i++) if (p_queue->queue[i].floor == etasje)
                {exist = 1;
                // sortQueue(p_queue, p_elevator);
                }  
                if (!exist) 
                {
                    QueueEntry entry = {etasje, buttontype};
                    if (etasje != elevio_floorSensor()) {
                        order_addToQueue(p_queue, entry);
                    }    
                    sortQueue(p_queue, p_elevator);
                }
            }
        }
    }
} 

//OLD CODE
void handler_run_v1()
{
    elevio_init();
    elevio_motorDirection(DIRN_STOP);
    Handler handler = {true};
    OrderQueue q = order_createQueue();
    Elevator elevator = elev_createElevator();

    elev_initElevator(&elevator);
    handler_resetLamps();
    int timer = 0;

    while(handler.go){
        
        if(elevio_stopButton())
        {
            elev_emergencyStop(&elevator);
            break;
        }
        
        processRequests(&q, &elevator);
        order_printQueue(&q);

        //Dersom ikke tom kø
        if ((q.queue[0].floor != -1) && (timer <= 0))
        {
            elev_moveTo(&elevator, q.queue[0].floor);
        } 
        else 
        {
            if (elevator.motor_dir != DIRN_STOP)
            {
                elev_setMotorDir(&elevator, DIRN_STOP);  
            }
        }
        

        //Elevator arrived
        if (elevator.has_arrived)
        {   
            printf("ELEVATOR : arrived at floor %i\n", elevator.current_floor);
            order_removeFromQueue(&q, elevator.current_floor);
            elev_openDoor(&elevator);
            timer += DOOR_OPEN_TIME_MS;
            elevator.has_arrived = false;
        } 
        
          
        //Door is ready to be closed
        if (timer <= 0 && elevator.door_open == true) 
        {
            if (elevio_obstruction()) 
            {
                timer += DOOR_OBS_TIME_MS;
                printf("ELEVATOR -> DOOR : obs!\n");
            }
            else
            {
                elev_closeDoor(&elevator);
                timer = 0;
            } 
        }

        //Decrement timer
        if (timer > 0) timer -= LOOP_DELAY_MS;
        nanosleep(&(struct timespec){0, LOOP_DELAY_MS*1000*1000}, NULL);        
    }
}