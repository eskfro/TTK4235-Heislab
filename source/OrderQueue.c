#include "../include/OrderQueue.h"

void order_clearQueue(OrderQueue* p_queue)
{
    for (int i = 0; i < QUEUESIZE; i++)
    {
        p_queue->queue[i].floor = -1;   
    }
}

void order_addToQueue(OrderQueue* p_queue, QueueEntry entry)
{
    elevio_buttonLamp(entry.floor, entry.dir, 1);

    for (int i = 0; i < QUEUESIZE; i++)
    {
        int q_floor = p_queue->queue[i].floor;
        int q_dir = p_queue->queue[i].dir;

        //Dersom floor er i Queue
        if (q_floor == entry.floor)
        {
            //Dersom entry er i Queue
            if (entry.dir == BUTTON_CAB) {
                p_queue->queue[i].dir = BUTTON_CAB;
                return;
            }
            else if (q_dir == BUTTON_CAB) return;
            else if (q_dir == entry.dir) return;
            else {
                p_queue->queue[i].dir = BUTTON_CAB; //BUTTON_CAB means dont care in this context
                return;
            }
        }

        //Setter inn i frammerste -1 verdi
        if (q_floor == -1)
        {
            p_queue->queue[i] = entry; 
            return;  
        } 
    }
}


void order_removeFromQueue(OrderQueue* p_queue, int floor)
{
    for (int i = 0; i < 3; i++) {
        elevio_buttonLamp(floor, i, 0);
    }

    // Iterate over the queue and shift elements left when the floor is found
    for (int i = 0; i < QUEUESIZE - 1; i++) {
        if (p_queue->queue[i].floor == floor) {
            // Shift the remaining elements to the left
            for (int j = i; j < QUEUESIZE - 1; j++) {
                p_queue->queue[j] = p_queue->queue[j + 1];
            }
            // Mark the last position as empty
            p_queue->queue[QUEUESIZE - 1].floor = -1;
            break; // Stop once the first match is removed
        }
    }
}

void order_printQueue(OrderQueue *p_queue)
{
    static OrderQueue last_queue; // Forrige kø

    int isQueueDifferent = 0;
    for (int i = 0; i < QUEUESIZE; i++) {
        if (p_queue->queue[i].floor != last_queue.queue[i].floor) {
            isQueueDifferent = 1;
            break;
        }
    }

    if (isQueueDifferent) {
        printf(" ---------- q : [");
        for (int i = 0; i < QUEUESIZE; i++) 
        {
            if (p_queue->queue[i].floor == -1) continue;
            printf("%i", p_queue->queue[i].floor);    
        }
        printf("] ---------\n");

        for (int i = 0; i < QUEUESIZE; i++) {
            last_queue.queue[i] = p_queue->queue[i];
        }
    }
}

OrderQueue order_createQueue()
{
    OrderQueue queue;
    order_clearQueue(&queue);
    return queue;
}
