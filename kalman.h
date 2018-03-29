/**
* kalman.h
*
*/
#ifndef KALMAN_H_
#define KALMAN_H_
typedef struct {
    float varVolt; // ������� ���������� (������������ � excel)
    float varProcess; // �������� ������� �� ���������
    // (����������� �������)
    float P;
    float Xe;
} kalman_t;
float kalman_filter(kalman_t * kalman, float value);
#endif