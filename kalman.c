/*
* kalman.c
*
*/
#include "kalman.h"


float kalman_filter(kalman_t * kalman, float value) {
    float Pc = kalman->P + kalman->varProcess;
    float G = Pc / (Pc + kalman->varVolt);
    kalman->P = (1 - G) * Pc;
    // "фильтрованное" значение
    kalman->Xe = G * (value - kalman->Xe) + kalman->Xe;
    return kalman->Xe;
}
