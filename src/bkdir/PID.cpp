#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;

    p_error = 0;
    i_error = 0;
    p_error = 0;

    update_counter = 0;
}

void PID::UpdateError(double cte) {
    d_error = cte-p_error;
    p_error = cte;
    i_error += cte;
    update_counter++;
}

double PID::TotalError() {
    float steer_value = -Kp*p_error - Ki*i_error - Kd*d_error;
    steer_value = steer_value<-1?-1:steer_value;
    steer_value = steer_value>1?1:steer_value;
    return steer_value;
}

