package org.firstinspires.ftc.teamcode.rasky.utilities.wrappers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.rasky.utilities.ControllerPID;

/**
 * Wrapper class for the Control Hub Gyroscope.
 * <p>
 * !! CALL INIT() METHOD BEFORE USING !!
 *
 * @author Lucian
 * @version 2.1
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
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
    }

    public BNO055IMU getGyro() {
        return imu;
    }

    Orientation angularOrientation;
    AngularVelocity angularVelocity;
    double updateInterval = 1;


    ElapsedTime oriUpdateTimer = new ElapsedTime();

    public void updateOrientation() {
        if (oriUpdateTimer.milliseconds() > updateInterval) {
            angularOrientation = imu.getAngularOrientation();
            oriUpdateTimer.reset();
        }
    }

    ElapsedTime velUpdateTimer = new ElapsedTime();

    public void updateVelocity() {
        if (velUpdateTimer.milliseconds() > updateInterval) {
            angularVelocity = imu.getAngularVelocity();
            velUpdateTimer.reset();
        }
    }

    /**
     * Set the minimum time before updating orientation and velocity.
     *
     * @param updateInterval Update interval in milliseconds.
     */
    public void setUpdateInterval(double updateInterval) {
        this.updateInterval = updateInterval;
    }

    public double getHeading() {
        return angularOrientation.firstAngle;
    }

    public double getForwardAngle() {
        return angularOrientation.secondAngle;
    }

    public double getLateralAngle() {
        return angularOrientation.thirdAngle;
    }

    //TODO: fix this :)
    public void initFirstAngles() {
        ElapsedTime initTime = new ElapsedTime();
        while (firstHeading == firstLateral && firstLateral == firstForward && initTime.milliseconds() < 2000) {
            updateOrientation();
            firstAngles = true;
            firstHeading = getHeading();
            firstLateral = getLateralAngle();
            firstForward = getForwardAngle();
        }
    }

    ControllerPID secondAnglePID = new ControllerPID(0.01, 0, 0);
    ControllerPID thirdAnglePID = new ControllerPID(0.01, 0, 0);

    public void showInfo(Telemetry telemetry) {
        double secondAngleResult = secondAnglePID.calculate(0, angularOrientation.secondAngle);
        double thirdAngleResult = thirdAnglePID.calculate(90, angularOrientation.thirdAngle);

        telemetry.addLine("---ANGLES---");
        telemetry.addData("First Angle: ", angularOrientation.firstAngle);
        telemetry.addData("Second Angle: ", angularOrientation.secondAngle);
        telemetry.addData("Third Angle: ", angularOrientation.thirdAngle);

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
