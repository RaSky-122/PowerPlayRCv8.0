package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * A basic PID controller.
 *
 * @author Lucian
 * @version 1.1
 */
public class ControllerPID {
    double kP, kI, kD;

    public ControllerPID(double Kp, double Ki, double Kd) {
        this.kP = Kp;
        this.kI = Ki;
        this.kD = Kd;
    }

    double prevError = 0;
    double integral = 0;
    public double calculate(double reference, double state) {
        double error = reference - state;

        double dt = getDt();
        double derivative = (error - prevError) / dt;

        integral += ((error + prevError) / 2) * dt;

        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    boolean started = false;
    ElapsedTime timer = new ElapsedTime();

    /**
     * This method is responsible for starting the PID controller timer correctly and
     * getting the time between the loops for the derivative value.
     *
     * @return Returns time between last and current loop
     */
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