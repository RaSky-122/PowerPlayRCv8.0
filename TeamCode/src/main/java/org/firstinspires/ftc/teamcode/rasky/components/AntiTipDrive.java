package org.firstinspires.ftc.teamcode.rasky.components;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rasky.utilities.ControllerPID;
import org.firstinspires.ftc.teamcode.rasky.utilities.DrivingMotors;
import org.firstinspires.ftc.teamcode.rasky.utilities.Gyroscope;

/**
 * Basic, Joystick based, mecanum drive with an anti tip controller.
 *
 * @author Lucian
 * @version 1.0
 */
public class AntiTipDrive {

    DrivingMotors motors;
    Gamepad gamepad;
    Gyroscope gyroscope;

    // Speed Multiplier
    double speed = 1.0;
    // If the Joystick has a lower value than this one the robot will not move
    final double controllerDeadzone = 0.15;
    // If the robot should move in reverse or not
    double reverse = 1.0;

    double Kp = 0.08, Ki = 0, Kd = 0;
    ControllerPID forwardAnglePID;
    ControllerPID lateralAnglePID;

    double forwardAngleValuePID = 0;
    double lateralAngleValuePID = 0;

    public AntiTipDrive(DrivingMotors motors, Gamepad gamepad, Gyroscope gyroscope) {
        this.motors = motors;
        this.gamepad = gamepad;
        this.gyroscope = gyroscope;
        forwardAnglePID = new ControllerPID(Kp, Ki, Kd);
        lateralAnglePID = new ControllerPID(Kp, Ki, Kd);
    }

    /**
     * Call this function asynchronously from the while in the OpMode
     */
    public void run() {

        forwardAngleValuePID = -forwardAnglePID.calculate(0.5,
                Math.toDegrees(gyroscope.getForwardAngle()));
        lateralAngleValuePID = lateralAnglePID.calculate(89,
                Math.toDegrees(gyroscope.getLateralAngle()));

        if (Math.abs(Math.toDegrees(gyroscope.getForwardAngle())) > 6)
            lateralAngleValuePID = 0;
        else if (Math.abs(Math.toDegrees(gyroscope.getLateralAngle())) > 6)
            forwardAngleValuePID = 0;
        else
            lateralAngleValuePID = forwardAngleValuePID = 0;


        double x = gamepad.left_stick_x + lateralAngleValuePID; // Strafe, Horizontal Axis
        double y = -gamepad.left_stick_y + forwardAngleValuePID; // Forward, Vertical Axis (joystick has +/- flipped)
        double r = gamepad.right_stick_x; // Rotation, Horizontal Axis

        x = addons(x) * reverse;
        y = addons(y) * reverse;
        r = addons(r);

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

    /**
     * Gets the coordinate of a joystick and verifies if it's after the controller deadzone.
     * Also calculates the result after the speed multiplier and returns it.
     *
     * @param coord The coordinate to make modifications to
     * @return The result after modifications
     */
    private double addons(double coord) {
        if (Math.abs(coord) < controllerDeadzone) {
            coord = 0;
        }
        coord = coord * speed;

        return coord;
    }

    public void setReverse(boolean isReverse) {
        if (isReverse) {
            reverse = -1.0;
        } else {
            reverse = 1.0;
        }
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Direction Multiplier: ", reverse);
        telemetry.addData("Speed Multiplier: ", speed);

        telemetry.addData("Forward Angle: ", Math.toDegrees(gyroscope.getForwardAngle()));
        telemetry.addData("Lateral Angle: ", Math.toDegrees(gyroscope.getLateralAngle()));
        telemetry.addData("Forward Angle PID: ", forwardAngleValuePID);
        telemetry.addData("Lateral Angle PID: ", lateralAngleValuePID);

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
