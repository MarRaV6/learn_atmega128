/**
* kalman.h
*
*/
#ifndef KALMAN_H_
#define KALMAN_H_
typedef struct {
    float varVolt; // среднее отклонение (определяется в excel)
    float varProcess; // скорость реакции на изменение
    // (подбирается вручную)
    float P;
    float Xe;
} kalman_t;
float kalman_filter(kalman_t * kalman, float value);
#endif