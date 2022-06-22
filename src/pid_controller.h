#ifndef pid_controller_h
#define pid_controller_h
#endif

class PID{

    public:

        PID(float kp, float ki, float kd, float dt);
        float computeControl(float error);

    private:
        float kp;
        float ki;
        float kd;
        float dt;
        float error_integral;
        float last_error;
};
