#include "pid_controller.h"

PID::PID(float kp, float ki, float kd, float dt){

    PID::kp = kp;
    PID::ki = ki;
    PID::kd = kd;
    PID::dt = dt;
    PID::last_error = 0;
    PID::error_integral = 0;

}

float PID::computeControl(float error){

    float error_derivative;
    float u; 
    
    // Update integral (Backwards Euler) //
    if(PID::ki != 0){
        PID::error_integral += error * PID::dt;
    }
    // Compute the derivative //
    if(PID::kd != 0){
        error_derivative = (error - last_error)/PID::dt;
    }

    // Update the last error //
    PID::last_error = error;

    // Saturate the output //
    u = u > 100 ? 100 : u;
    u = u < -100 ? -100 : u; 

    return kp * error + ki * PID::error_integral + kd * error_derivative;

}