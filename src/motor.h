#ifndef motor_h
#define motor_h
#endif

#define FORWARD TRUE
#define BACKWARDS FALSE



class Motor{

    /* Motor class for driving a motor using a driver L298N or similar */
    public:
        // Constructor 
        Motor(unsigned char pwm_pin, unsigned char dir_pin_a, unsigned char dir_pin_b, float dead_zone, bool reversed = false);
        void setValue(float value);
        void setBreak();
        void stopMotor();
    
    private:
        unsigned char pwm_pin;
        unsigned char dir_pin_b;
        unsigned char dir_pin_a;
        float dead_zone;
        bool reversed;




}