#pragma once

#include <stdbool.h>
#include <stdio.h>
#include <assert.h>

#define N_FLOORS 4

typedef struct {
    int hall_up;
    int hall_down;
    int cab;
} FloorType;


typedef struct {
    FloorType list[4];
} Matrix;


Matrix matrix_createMatrix();

void matrix_clearMatrix(Matrix* m);
void matrix_clearFloor(Matrix* m, int floor);
void matrix_printMatrix(Matrix* m);

int matrix_isEmpty(Matrix* m);
int matrix_isCallFromFloor(Matrix* m, int floor);


