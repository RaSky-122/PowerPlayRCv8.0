package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * A basic PID controller.
 *
 * @author Lucian
 * @version 1.0
 */
public class ControllerPID {
    double prevError = 0;
    double integral = 0;
    boolean started = false;

    ElapsedTime timer = new ElapsedTime();

    double Kp, Ki, Kd;

    public ControllerPID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double calculate(double reference, double state) {
        double error = reference - state;
        double dt = getDt();
        double derivative = (error - prevError) / dt;
        integral += ((error + prevError) / 2) * dt;
        prevError = error;
        return Kp * error + Ki * integral + Kd * derivative;
    }

    public double calculate(double error) {
        return calculate(0, -error);
    }

    private double getDt() {
        if (!started) {
            started = true;
            timer.reset();
        }
        double dt = timer.seconds();
        timer.reset();
        return dt;
    }


}