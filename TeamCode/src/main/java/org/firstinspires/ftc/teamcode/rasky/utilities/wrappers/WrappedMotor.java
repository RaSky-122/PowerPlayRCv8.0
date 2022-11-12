package org.firstinspires.ftc.teamcode.rasky.utilities.wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.rasky.utilities.ControllerPID;

/**
 * This class is made as a wrapper for the the motors to be able to more easily initialize and
 * run them to a given position.
 *
 * @author Lucian
 * @version 1.2
 */
public class WrappedMotor {
    public DcMotorEx motor;
    HardwareMap hardwareMap;
    VoltageSensor voltageSensor;

    public WrappedMotor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initialization method.
     *
     * @param name       The name of the motor
     * @param isReversed If the direction of the motor is reversed or not
     * @param isPID      If the motor should be controlled by a PID controller or not
     * @param brakes     If the motor should brake on 0 power or not
     */
    public void Init(String name, boolean isReversed, boolean isPID, boolean brakes) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.setDirection(isReversed);
        this.setPIDMode(isPID);
        this.setBrakes(brakes);
    }


    public double targetPosition = 0;
    public double currentPosition = 0;

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    double tolerance = 0;

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double direction = 1;

    /**
     * Call this method asynchronously to update the lift's motor position.
     */
    public void updatePosition() {
        currentPosition = motor.getCurrentPosition() * encoderDirection;

        if (currentPosition <= targetPosition)
            direction = 1;
        else
            direction = -1;

        if (voltageCompensated)
            voltageCompensation = 12 / voltageSensor.getVoltage();
        else
            voltageCompensation = 1;


        if (Math.abs(targetPosition - currentPosition) > tolerance) {
            if (positionPIDMode)
                motor.setPower(-positionPID.calculate(currentPosition, targetPosition) * speed * voltageCompensation);
            else
                motor.setPower(speed * direction * voltageCompensation);
        } else if (hold)
            motor.setPower(0.1);
        else
            motor.setPower(0);

    }

    boolean voltageCompensated = false;
    double voltageCompensation = 1;

    public void setVoltageCompensated(boolean voltageCompensated) {
        this.voltageCompensated = voltageCompensated;
    }

    boolean positionPIDMode = false;
    ControllerPID positionPID = new ControllerPID(0, 0, 0);

    public void setPositionPIDMode(boolean positionPIDMode) {
        this.positionPIDMode = positionPIDMode;
    }

    public void setPositionPID(double kP, double kI, double kD) {
        positionPID = new ControllerPID(kP, kI, kD);
    }


    public boolean isBusy() {
        if (Math.abs(targetPosition - currentPosition) > tolerance)
            return true;
        else
            return false;
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double encoderDirection = 1;

    public void setEncoderDirection(double encoderDirection) {
        this.encoderDirection = encoderDirection;
    }

    double speed = 1;

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    boolean hold = false;

    public void holdMode(boolean holdMode) {
        hold = holdMode;
    }

    public void setPIDMode(boolean isPID) {
        if (isPID)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setDirection(boolean isReversed) {
        if (isReversed)
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setBrakes(boolean brakes) {
        if (brakes)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
