#include "src/motor.h"
#include "src/pid_controller.h"
#include "Arduino.h"

#define MOTOR_LEFT_PWM 0
#define MOTOR_LEFT_IN1 0
#define MOTOR_LEFT_IN2 0
#define MOTOR_RIGHT_PWM 0
#define MOTOR_RIGHT_IN1 0
#define MOTOR_RIGHT_IN2 0
#define DEAD_ZONE 50

#define MOTOR_LEFT_ENCODER_A 0
#define MOTOR_LEFT_ENCODER_B 0
#define MOTOR_RIGHT_ENCODER_A 0
#define MOTOR_RIGHT_ENCODER_B 0

#define LOOP_TIME 10
#define TICS_PER_TURN 39

#define ROBOT_WHEEL_RADIUS 0.5
#define ROBOT_DISTANCE 0.04

#define RIGHT_KP 0.1
#define RIGHT_KI 0.01
#define RIGHT_KD 0.0




// Create the motors objects //
motor_left = Motor(MOTOR_LEFT_PWM, MOTOR_LEFT_IN1, MOTOR_LEFT_PWM, DEAD_ZONE);
motor_right = Motor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_IN1, MOTOR_RIGHT_PWM, DEAD_ZONE);

double encoder_left_count = 0;
double encoder_right_count = 0;
double encoder_left_last_count = 0;
double encoder_right_last_count = 0;

unsigned long lastTime = 0

float target_wheel_speeds[2];
float sensed_wheel_speeds[2];
float target_robot_speeds[2];

// PID objects //
PID leftPID(RIGHT_KP, RIGHT_KI, RIGHT_KD, LOOP_TIME/1000);
PID rightPID(RIGHT_KP, RIGHT_KI, RIGHT_KD, LOOP_TIME/1000);

void setup(){

    // Set the encoders interruptions //
    attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENCODER_A), left_encoder_A_interrupt_routine, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_ENCODER_B), left_encoder_B_interrupt_routine, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENCODER_A), right_encoder_A_interrupt_routine, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_ENCODER_B), right_encoder_B_interrupt_routine, CHANGE);

}

void loop(){

    unsigned long currentTime = millis();
    float left_motor_speed:
    float right_motor_speed;
    float u_right, u_left;

    if(currentTime - lastTime >= LOOP_TIME){

        // Update the last time //
        lastTime = currentTime;
        // Compute the speeds //
        sensed_wheel_speeds[1] = (encoder_left_count - encoder_left_last_count) / TICS_PER_TURN;
        sensed_wheel_speeds[0] = (encoder_right_count - encoder_right_last_count) / TICS_PER_TURN;
        encoder_left_last_count = encoder_left_count;
        encoder_right_last_count = encoder_right_count;

        // Compute the desired wheel speeds from V and W //
        update_target_speeds(target_robot_speeds, target_wheel_speeds);

        // Compute the value from the PID //
        u_right = rightPID.computeControl(target_wheel_speeds[0] - sensed_wheel_speeds[0])
        u_left = rightPID.computeControl(target_wheel_speeds[1] - sensed_wheel_speeds[1])

        // Apply the control signals //
        motor_right.setValue(u_right);
        motor_left.setValue(u_left);

    }

} 

void update_target_speeds(float target_robot_speeds[2], float target_wheel_speeds[2]){
    /* Compute the inverse kinematics */
    target_wheel_speeds[0] = (ROBOT_DISTANCE/2 * target_robot_speeds[1] + target_robot_speeds[0])/ROBOT_WHEEL_RADIUS
    target_wheel_speeds[1] = (-ROBOT_DISTANCE/2 * target_robot_speeds[1] + target_robot_speeds[0])/ROBOT_WHEEL_RADIUS
}

void left_encoder_A_interrupt_routine(){

    if (digitalRead(MOTOR_LEFT_ENCODER_A) == HIGH) {   
        if (digitalRead(MOTOR_LEFT_ENCODER_B) == HIGH) {  
        encoder_left_count++;        // CW
        } else {encoder_left_count--;}         // CCW
    } else { 
        if (digitalRead(MOTOR_LEFT_ENCODER_B) == LOW) {   
        encoder_left_count++;          // CW
        } else {encoder_left_count--;}          // CCW
    }
}

void left_encoder_B_interrupt_routine(){

    if (digitalRead(MOTOR_LEFT_ENCODER_B) == HIGH) {   
        if (digitalRead(MOTOR_LEFT_ENCODER_A) == HIGH) {  
        encoder_left_count++;        // CW
        } else {encoder_left_count--;}         // CCW
    } else { 
        if (digitalRead(MOTOR_LEFT_ENCODER_A) == LOW) {   
        encoder_left_count++;          // CW
        } else {encoder_left_count--;}          // CCW
    }
}

void right_encoder_A_interrupt_routine(){

    if (digitalRead(MOTOR_RIGHT_ENCODER_A) == HIGH) {   
        if (digitalRead(MOTOR_RIGHT_ENCODER_B) == HIGH) {  
        encoder_right_count++;        // CW
        } else {encoder_right_count--;}         // CCW
    } else { 
        if (digitalRead(MOTOR_RIGHT_ENCODER_B) == LOW) {   
        encoder_right_count++;          // CW
        } else {encoder_right_count--;}          // CCW
    }
}

void right_encoder_B_interrupt_routine(){

    if (digitalRead(MOTOR_RIGHT_ENCODER_B) == HIGH) {   
        if (digitalRead(MOTOR_RIGHT_ENCODER_A) == HIGH) {  
        encoder_right_count++;        // CW
        } else {encoder_right_count--;}         // CCW
    } else { 
        if (digitalRead(MOTOR_RIGHT_ENCODER_A) == LOW) {   
        encoder_right_count++;          // CW
        } else {encoder_right_count--;}          // CCW
    }

}