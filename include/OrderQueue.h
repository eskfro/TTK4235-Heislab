#pragma once

#include <stdio.h>
#include "../source/driver/elevio.h"

#define QUEUESIZE 4

typedef struct {
    int floor;
    ButtonType dir;
} QueueEntry;

typedef struct {
    QueueEntry queue[QUEUESIZE];
} OrderQueue;


void order_clearQueue(OrderQueue* p_queue);
void order_addToQueue(OrderQueue* p_queue, QueueEntry entry); 
void order_removeFromQueue(OrderQueue* p_queue, int floor);
void order_printQueue(OrderQueue* p_queue);
OrderQueue order_createQueue();