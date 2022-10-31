package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Wrapper class for the Control Hub Gyroscope.
 *
 * @author Lucian
 * @version 1.2
 */
public class Gyroscope {
    BNO055IMU imu;
    HardwareMap hardwareMap;

    public double firstHeading = 0;
    public double firstLateral = 0;
    public double firstForward = 0;

    boolean firstAngles = false;

    public Gyroscope(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    public void Init() {

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public BNO055IMU getGyro() {
        return imu;
    }

    public double getHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    public double getForwardAngle() {
        return imu.getAngularOrientation().secondAngle * -1;
    }

    public double getLateralAngle() {
        return imu.getAngularOrientation().thirdAngle;
    }

    //Not working, idk why
    public void initFirstAngles() {
        if (!firstAngles) {
            firstAngles = true;
            firstHeading = getHeading();
            firstLateral = getLateralAngle();
            firstForward = getForwardAngle();
        }
    }

    ControllerPID secondAnglePID = new ControllerPID(0.01, 0, 0);
    ControllerPID thirdAnglePID = new ControllerPID(0.01, 0, 0);

    public void showAllInfo(Telemetry telemetry) {
        Orientation angularOrientation = imu.getAngularOrientation();
        AngularVelocity angularVelocity = imu.getAngularVelocity();

        angularOrientation.secondAngle = angularOrientation.secondAngle * -1;

        double secondAngleResult = -secondAnglePID.calculate(0, Math.toDegrees(angularOrientation.secondAngle));
        double thirdAngleResult = thirdAnglePID.calculate(90, Math.toDegrees(angularOrientation.thirdAngle));

        telemetry.addLine("---ANGLES---");
        telemetry.addData("First Angle: ", Math.toDegrees(angularOrientation.firstAngle));
        telemetry.addData("Second Angle: ", Math.toDegrees(angularOrientation.secondAngle));
        telemetry.addData("Third Angle: ", Math.toDegrees(angularOrientation.thirdAngle));

        telemetry.addLine("---PID---");
        telemetry.addData("Second Angle PID: ", secondAngleResult);
        telemetry.addData("Third Angle PID: ", thirdAngleResult);

        telemetry.addLine("---VELOCITY---");
        telemetry.addData("X Rotation Rate: ", angularVelocity.xRotationRate);
        telemetry.addData("Y Rotation Rate: ", angularVelocity.yRotationRate);
        telemetry.addData("Z Rotation Rate: ", angularVelocity.zRotationRate);
    }

    ;

}
