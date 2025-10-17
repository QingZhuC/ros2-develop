#ifndef PID_CONTROL_HPP
#define PID_CONTROL_HPP

#define MAX_IOUT 300.f
#define MAX_OUT  1000.f
class PID {
private:
    double target;
    double error_old, error_new;
    double KP;
    double KI;
    double KD;
    double pout;
    double iout;
    double dout;
    double out;

public:
    void motor_PID_Init(
        double speed_Init, double target_Init, double KP_Init, double KI_Init, double KD_Init);
    void change_target(double target_Init) { target = target_Init; }
    void PID_calc(double last_data);
    void limit_iout();
    void limit_out();
    double get_out();
};

void PID::motor_PID_Init(
    double speed_Init, double target_Init, double KP_Init, double KI_Init, double KD_Init) {
    target = target_Init;
    KP = KP_Init;
    KI = KI_Init;
    KD = KD_Init;
    error_old = pout = iout = dout = 0.0f;
    error_new = target - speed_Init;
}

void PID::PID_calc(double last_data) // PID算法实现
{
    error_old = error_new;
    error_new = target - last_data;
    pout = KP * error_new;
    iout += KI * error_new;
    limit_iout();
    dout = KD * (error_new - error_old);
    out = pout + iout + dout;
    limit_out();
}
void PID::limit_iout() {
    if (iout > MAX_IOUT)
        iout = MAX_IOUT;
}
void PID::limit_out() {
    if (out > MAX_OUT)
        out = MAX_OUT;
}
double PID::get_out() { return out; }

#endif