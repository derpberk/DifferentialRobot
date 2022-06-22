#include "Arduino.h"
#include "motor.h"

Motor::Motor(unsigned char pwm_pin, unsigned char dir_pin_a, unsigned char dir_pin_b, float dead_zone, bool reversed = false){

    // Declare pins 
    pinMode(pwm_pin, OUTPUT);
    pinMode(dir_pin_a, OUTPUT);
    pinMode(dir_pin_b, OUTPUT);

    // Set the pin values //
    Motor::dir_pin_a = dir_pin_a;
    Motor::dir_pin_B = dir_pin_b;
    Motor::pwm_pin = pwm_pin;
    Motor::reversed = reversed;
    Motor::dead_zone = dead_zone

    // Stop motor
    Motor::stopMotor();

}

Motor::stopMotor(){

    digitalWrite(Motor::dir_pin_a, LOW);
    digitalWrite(Motor::dir_pin_b, LOW);
    analogWrite(Motor::pwm_pin, 0);

}

Motor::setBreak(){
    /* Reverse current for 1 ms and stop */
        
    digitalWrite(Motor::dir_pin_a, HIGH);
    digitalWrite(Motor::dir_pin_b, HIGH);
    analogWrite(Motor::pwm_pin, 0);
    delay(1);
    Motor::stopMotor();

}

Motor::setValue(float value){
    /* Write the value for the motor */

    float abs_value = (value >= 0) ? value : -value;
    
    if(value > 0){
        Motor::setDirection(FORWARD);
    }else{
        Motor::setDirection(BACKWARDS);
    }
    analogWrite(Motor::pwm_pin, Motor::dead_zone + (abs_value*(255 - Motor::dead_zone)));

}

Motor::setDirection(bool dir){

    if(dir & not Motor::reversed){
        pinMode(Motor::dir_pin_a, HIGH);
        pinMode(Motor::dir_pin_b, LOW);
    }else{
        pinMode(Motor::dir_pin_a, LOW);
        pinMode(Motor::dir_pin_b, HIGH);
    }

}