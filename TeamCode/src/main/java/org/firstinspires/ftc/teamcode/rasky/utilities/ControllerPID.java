package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * A basic PID controller.
 *
 * @author Lucian
 * @version 1.2
 */
public class ControllerPID {
    double kP, kI, kD;

    public ControllerPID(double Kp, double Ki, double Kd) {
        this.kP = Kp;
        this.kI = Ki;
        this.kD = Kd;
    }

    double lastError = 0;
    double lastReference = 0;
    double integral = 0;

    public double calculate(double reference, double currentState) {
        double error = reference - currentState;

        double loopTime = getLoopTime();
        double derivative = (error - lastError) / loopTime;

        if (lastReference != reference) {
            integral = 0;
        }
        integral += ((error + lastError) / 2) * loopTime;

        lastError = error;
        lastReference = reference;
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
    private double getLoopTime() {
        if (!started) {
            started = true;
            timer.reset();
        }
        double time = timer.seconds();
        timer.reset();
        return time;
    }


}