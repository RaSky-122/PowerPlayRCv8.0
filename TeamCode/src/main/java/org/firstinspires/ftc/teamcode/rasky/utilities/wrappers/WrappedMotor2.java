package org.firstinspires.ftc.teamcode.rasky.utilities.wrappers;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
 * @version 2.0
 */
public class WrappedMotor2 {
    public DcMotorEx motor;
    HardwareMap hardwareMap;
    VoltageSensor voltageSensor;

    public WrappedMotor2(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initialization method.
     *
     * @param name             The name of the motor
     * @param isReversed       If the direction of the motor is reversed or not
     * @param velocityPIDFMode If the motor should be controlled by a PID controller or not
     * @param positionPIDMode  If the motor position should be controlled by a PID controller or not
     * @param brakes           If the motor should brake on 0 power or not
     */
    public void Init(String name, boolean isReversed, boolean velocityPIDFMode, boolean positionPIDMode, boolean brakes, boolean reset) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        if (reset)
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        this.setDirection(isReversed);
//        this.setPositionPIDMode(positionPIDMode);
//        this.setVelocityPIDFMode(velocityPIDFMode);
        motor.setTargetPosition(0);
        motor.setTargetPositionTolerance(10);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.setBrakes(brakes);
    }

    PIDCoefficients coeffs = new PIDCoefficients(8, 3, 0);
    PIDFController controller = new PIDFController(coeffs, 0, 0);

    /**
     * Call this method asynchronously to update the lift's motor position.
     */
    public void updatePosition() {

        controller.setTargetPosition(targetPosition);

        if (motor.isBusy())
            motor.setPower(1);
        else
            motor.setPower(0.1);
    }

    // VOLTAGE RELATED STUFF

    boolean voltageCompensated = false;
    double voltageCompensation = 1;

    public void setVoltageCompensated(boolean voltageCompensated) {
        this.voltageCompensated = voltageCompensated;
    }

    public double getVoltageCompensation() {
        if (voltageCompensated)
            return (12 / voltageSensor.getVoltage());
        else
            return 1;
    }

    // POSITION RELATED STUFF

    boolean positionPIDMode = false;
    ControllerPID positionPID = new ControllerPID(0, 0, 0);

    public void setPositionPIDMode(boolean positionPIDMode) {
        this.positionPIDMode = positionPIDMode;
    }

    public void setPositionPID(double kP, double kI, double kD) {
        positionPID = new ControllerPID(kP, kI, kD);
    }

    public double targetPosition = 0;
    public double currentPosition = 0;

    public void setTargetPosition(double targetPosition) {
        motor.setTargetPosition((int) targetPosition);
    }

    double tolerance = 0;

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double direction = 1;

    double getDirection() {
        if (currentPosition <= targetPosition)
            return 1;
        else
            return -1;
    }

    public boolean isBusy() {
        return (Math.abs(targetPosition - currentPosition) > tolerance);
    }

    // POWER RELATED STUFF

    public void setPower(double power) {
        motor.setPower(power);
    }

    public double getPower() {
        return motor.getPower();
    }

    boolean hold = false;
    double gravityCounter = 0.1;

    public void holdMode(boolean holdMode) {
        hold = holdMode;
    }

    double speedMultiplier = 1;

    public void setSpeedMultiplier(double speedMultiplier) {
        this.speedMultiplier = speedMultiplier;
    }

    //VELOCITY RELATED STUFF

    public void setVelocityPIDFMode(boolean velocityPIDMode) {
        if (velocityPIDMode)
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        else
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setVelocityPIDF(double kP, double kI, double kD, double kF) {
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    //MOTOR UTILITIES STUFF

    public void setDirection(boolean isReversed) {
        if (isReversed)
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public double encoderDirection = 1;

    @Deprecated
    public void setEncoderDirection(double encoderDirection) {
        this.encoderDirection = encoderDirection;
    }

    public void setBrakes(boolean brakes) {
        if (brakes)
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        else
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
}
