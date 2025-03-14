#include "../include/OrderQueue.h"

void order_clearQueue(OrderQueue* p_queue)
{
    for (int i = 0; i < QUEUESIZE; i++) p_queue->queue[i].floor = -1;   
}


void order_removeFromQueue(OrderQueue* p_queue, int floor)
{
    for (int i = 0; i < 3; i++) {
        elevio_buttonLamp(floor, i, 0);
    }

    for (int i = 0; i < QUEUESIZE - 1; i++) {
        if (p_queue->queue[i].floor == floor) {
            // Shift left
            for (int j = i; j < QUEUESIZE - 1; j++) {
                p_queue->queue[j] = p_queue->queue[j + 1];
            }
            p_queue->queue[QUEUESIZE - 1].floor = -1;
            break;
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
