#include "../include/Matrix.h"


void matrix_printMatrix(Matrix* m) 
{
    static Matrix lastMatrix;
    int is_matrix_different = 0;
    for (int i = 0; i < N_FLOORS; i++) {
        if (m->list[i].hall_up != lastMatrix.list[i].hall_up || 
            m->list[i].hall_down != lastMatrix.list[i].hall_down ||
            m->list[i].cab != lastMatrix.list[i].cab)
        {
            is_matrix_different = 1;
            break;
        }
    }

    if (is_matrix_different)
    {   
        printf("\n");
        printf(" ---------- up | down | cab -\n");
        for (int i = 0; i < N_FLOORS; i++) 
        {
            printf("floor %i :", N_FLOORS - 1 - i);
            printf("    %i     %i     %i  |\n", m->list[3-i].hall_up, m->list[3-i].hall_down, m->list[3-i].cab);
        }
        printf("\n");

        for (int i = 0; i < N_FLOORS; i++) {
            lastMatrix.list[i].hall_up = m->list[i].hall_up;
            lastMatrix.list[i].hall_down = m->list[i].hall_down;
            lastMatrix.list[i].cab = m->list[i].cab;
        }
    }
}

Matrix matrix_createMatrix() 
{
    FloorType f0 = {0, 0, 0};
    FloorType f1 = {0, 0, 0};
    FloorType f2 = {0, 0, 0};
    FloorType f3 = {0, 0, 0};

    Matrix m = {f0, f1, f2, f3};
    return m;
}



void matrix_clearMatrix(Matrix* m)
{
    for (int i = 0; i < N_FLOORS; i++) 
    {
        m->list[i].hall_up = 0;
        m->list[i].hall_down = 0;
        m->list[i].cab = 0;
    }
}


void matrix_clearFloor(Matrix* m, int floor) 
{
    m->list[floor].hall_up = 0;
    m->list[floor].hall_down = 0;
    m->list[floor].cab = 0;
}



int matrix_isEmpty(Matrix *m)
{
    for (int i = 0; i < N_FLOORS; i++)
    {
        if (m->list[i].cab == 1 || m->list[i].hall_up == 1 || m->list[i].hall_down == 1)
        {
            return 0;
        }
    }
    return 1;
}

int matrix_isCallFromFloor(Matrix* m, int floor) 
{
    assert(floor >= 0 && floor < N_FLOORS);
    return (int)(m->list[floor].cab || m->list[floor].hall_up || m->list[floor].hall_down);
}


