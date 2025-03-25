#include "PID.h"
#include <iostream>
using namespace std;
PID::PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    // �������
    float error = target - current;
    // �����ۼ�
    intergral += error;
    // ����������һ�����Ĳ���
    derivative = error - prevError;
    // ����pid�Ĺ�ʽ
    targetpoint = kp * error + ki * intergral + kd * derivative;
    // ��¼��һ�ε����
    prevError = error;
    return targetpoint;
}

void PID::reset() {
    targetpoint = 0;
    intergral = 0;
    derivative = 0;
    prevError = 0;
}

void PID::Set_PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
