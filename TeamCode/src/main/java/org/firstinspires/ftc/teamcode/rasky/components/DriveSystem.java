package org.firstinspires.ftc.teamcode.rasky.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rasky.utilities.ControllerPID;
import org.firstinspires.ftc.teamcode.rasky.utilities.DrivingMotors;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Gyroscope;

/**
 * All in one Drive system.
 * <p>
 * Contains:
 * <p>
 * - Robot Centric Drive ( the robot moves relative to it's heading )
 * <p>
 * - Field Centric Drive ( the robot moves relative to the driver )
 * <p>
 * - Anti Tip Mode ( if the robot is going to tip it turns itself back over )
 *
 * @author Lucian
 * @version 1.0
 */
public class DriveSystem {

    DrivingMotors motors;
    Gamepad gamepad;
    Gyroscope gyroscope;
    HardwareMap hardwareMap;
    VoltageSensor voltageSensor;

    double reverse = 1.0; // If the robot should move in reverse or not
    double strafeCorrection = 1.1; // Strafe speed correction due to mecanum strafing not being the same as forward

    //Anti Tip Drive constants and PIDs
    double Kp = 0.08, Ki = 0, Kd = 0;
    ControllerPID forwardAnglePID;
    ControllerPID lateralAnglePID;

    double forwardAngleValuePID = 0;
    double lateralAngleValuePID = 0;

    public DriveSystem(DrivingMotors motors, Gamepad gamepad, Gyroscope gyroscope, HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.motors = motors;
        this.gamepad = gamepad;
        this.gyroscope = gyroscope;

        forwardAnglePID = new ControllerPID(Kp, Ki, Kd);
        lateralAnglePID = new ControllerPID(Kp, Ki, Kd);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        gyroscope.updateOrientation();
        gyroscope.initFirstAngles();
    }

    double x, y, r;

    /**
     * Call this function asynchronously from the while in the OpMode
     */
    public void run() {
        double voltageCompensation = 12 / voltageSensor.getVoltage();
        x = gamepad.left_stick_x; // Strafe, Horizontal Axis
        y = -gamepad.left_stick_y; // Forward, Vertical Axis (joystick has +/- flipped)
        r = gamepad.right_stick_x; // Rotation, Horizontal Axis

        // Update Gyroscope only when needed so it doesn't increase loop times by a lot
        if (fieldCentricMode || antiTipMode) {
            gyroscope.updateOrientation();
        }

        if (fieldCentricMode) {
            rotateJoystick2D();
        }

        if (antiTipMode) {
            calculateTippingPID();
        } else {
            lateralAngleValuePID = forwardAngleValuePID = 0;
        }

        addons();
        x = (x + lateralAngleValuePID) * voltageCompensation;
        y = (y + forwardAngleValuePID) * voltageCompensation;
        r = r * voltageCompensation;

        /*
        If the rotation and forward direction are both engaged the value can go past 1.0 (100%).
        To prevent that we implement a denominator that normalizes the values to 100% max.
         */
        double normalizer = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(r), 1.0);

        double leftFrontPower = (y + x + r) / normalizer;
        double rightFrontPower = (y - x - r) / normalizer;
        double leftRearPower = (y - x + r) / normalizer;
        double rightRearPower = (y + x - r) / normalizer;

        motors.leftFront.setPower(leftFrontPower);
        motors.rightFront.setPower(rightFrontPower);
        motors.leftRear.setPower(leftRearPower);
        motors.rightRear.setPower(rightRearPower);
    }

    double speed = 1.0; // Speed Multiplier
    final double controllerDeadzone = 0.15; // If the Joystick has a lower value than this one the robot will not move

    /**
     * Calculates the following for each joystick coordinate:
     * <p>
     * - Checks if it's enough to pass the deadzone
     * <p>
     * - Modifies based on the speed multiplier
     * <p>
     * - Corrects strafing speed
     * <p>
     * -Reverses direction depending on the case
     */
    private void addons() {
        // Strafe modifications
        if (Math.abs(x) < controllerDeadzone) {
            x = 0;
        }
        x = x * speed * strafeCorrection * reverse;

        if (Math.abs(y) < controllerDeadzone) {
            y = 0;
        }
        y = y * speed * reverse;

        if (Math.abs(r) < controllerDeadzone) {
            r = 0;
        }
        r = r * speed;
    }

    /**
     * Calculates the tipping of the robot and the PID correction for it.
     */
    private void calculateTippingPID() {
        forwardAngleValuePID = forwardAnglePID.calculate(gyroscope.firstForward, gyroscope.getForwardAngle());
        lateralAngleValuePID = lateralAnglePID.calculate(gyroscope.firstLateral, gyroscope.getLateralAngle());

        if (Math.abs(gyroscope.getForwardAngle() - gyroscope.firstForward) > 5) {
            lateralAngleValuePID = 0;
        } else if (Math.abs(gyroscope.getLateralAngle() - gyroscope.firstLateral) > 5) {
            forwardAngleValuePID = 0;
        } else {
            lateralAngleValuePID = forwardAngleValuePID = 0;
        }
    }

    private void rotateJoystick2D() {
        double headingOffset = -Math.toRadians(gyroscope.getHeading());

        // Rotating the vector of the joystick in a 2D space based on heading offset
        // See this to understand the formulas: https://matthew-brett.github.io/teaching/rotation_2d.html
        x = x * Math.cos(headingOffset) - y * Math.sin(headingOffset);
        y = x * Math.sin(headingOffset) + y * Math.cos(headingOffset);
    }

    /**
     * Reverses the front of the robot with the back of the robot
     * ONLY IN ROBOT CENTRIC DRIVE
     *
     * @param isReverse Boolean value to set the reverse on/off
     */
    public void setReverse(boolean isReverse) {
        if (isReverse) {
            reverse = -1.0;
        } else {
            reverse = 1.0;
        }
    }

    boolean antiTipMode = false;

    public void setAntiTipMode(boolean antiTipMode) {
        this.antiTipMode = antiTipMode;
    }

    boolean fieldCentricMode = false;

    public void setFieldCentricMode(boolean fieldCentricMode) {
        if (fieldCentricMode != this.fieldCentricMode) {
            reverse = 1.0;
        }
        this.fieldCentricMode = fieldCentricMode;
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", speed);

        if (fieldCentricMode) {
            telemetry.addData("Robot Angle: ", gyroscope.getHeading());
        }

        if (antiTipMode) {
            telemetry.addData("Forward Angle: ", gyroscope.getForwardAngle());
            telemetry.addData("Lateral Angle: ", gyroscope.getLateralAngle());
            telemetry.addData("Forward Angle PID: ", forwardAngleValuePID);
            telemetry.addData("Lateral Angle PID: ", lateralAngleValuePID);
        }

        telemetry.addData("LeftRear Position: ", motors.leftRear.getCurrentPosition());
        telemetry.addData("RightRear Position: ", motors.rightRear.getCurrentPosition());
        telemetry.addData("LeftFront Position: ", motors.leftFront.getCurrentPosition());
        telemetry.addData("RightFront Position: ", motors.rightFront.getCurrentPosition());

        telemetry.addData("LeftRear Power: ", motors.leftRear.getPower());
        telemetry.addData("RightRear Power: ", motors.rightRear.getPower());
        telemetry.addData("LeftFront Power: ", motors.leftFront.getPower());
        telemetry.addData("RightFront Power: ", motors.rightFront.getPower());
    }

}
