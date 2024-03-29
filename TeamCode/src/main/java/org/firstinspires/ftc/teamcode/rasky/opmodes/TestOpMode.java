package org.firstinspires.ftc.teamcode.rasky.opmodes;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptGamepadRumble;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.rasky.components.AntiTipDrive;
import org.firstinspires.ftc.teamcode.rasky.components.DriveSystem;
import org.firstinspires.ftc.teamcode.rasky.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.rasky.components.LiftClaw;
import org.firstinspires.ftc.teamcode.rasky.components.LiftNewClaw;
import org.firstinspires.ftc.teamcode.rasky.components.LiftSystem;
import org.firstinspires.ftc.teamcode.rasky.learning.TestClass;
import org.firstinspires.ftc.teamcode.rasky.utilities.LoopTimeMeasure;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;
import org.firstinspires.ftc.teamcode.rasky.utilities.DrivingMotors;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Gyroscope;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrapperDistanceSensor;

/**
 * The test TeleOP program for testing new features.
 *
 * @author Lucian
 * @version 1.2
 */
@TeleOp(name = "Test OpMode", group = Constants.testGroup)
public class TestOpMode extends LinearOpMode {

    DrivingMotors motors;
    Gyroscope gyroscope;

    DriveSystem driveSystem;
    LiftSystem liftSystem;
    LiftNewClaw liftNewClaw;

    Gamepad drivingGamepad;
    Gamepad utilityGamepad;

    //WrapperDistanceSensor distanceSensorLeft;
    //WrapperDistanceSensor distanceSensorRight;

    Gamepad.RumbleEffect endgameRumble;

    @Override
    public void runOpMode() throws InterruptedException {
        //PhotonCore.enable();

        //Set the gamepads to the desired gamepad
        drivingGamepad = gamepad1;
        utilityGamepad = gamepad2;
        endgameRumble = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 500)
                .build();

        motors = new DrivingMotors(hardwareMap);
        motors.Init();

        liftSystem = new LiftSystem(hardwareMap, utilityGamepad);
        liftSystem.Init();

        liftNewClaw = new LiftNewClaw(hardwareMap, utilityGamepad);
        liftNewClaw.Init();

        gyroscope = new Gyroscope(hardwareMap);
        gyroscope.Init();

        driveSystem = new DriveSystem(motors, drivingGamepad, gyroscope, hardwareMap);

        //distanceSensorLeft = new WrapperDistanceSensor(hardwareMap, "sensorLeft");
        // distanceSensorRight = new WrapperDistanceSensor(hardwareMap, "sensorRight");

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            //telemetry.addData("Right Sensor Distance: ", distanceSensorRight.getDistance());
            //telemetry.addData("Right Sensor Distance Avg: ", distanceSensorRight.getAverage());
            //distanceSensorRight.update();
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        Button driveModeButton = new Button();
        driveSystem.setAntiTipMode(false);

        LoopTimeMeasure loopTime = new LoopTimeMeasure(telemetry);

        ElapsedTime opModeTimer = new ElapsedTime();
        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {
            if (opModeTimer.seconds() >= 85 && opModeTimer.seconds() <= 85.5) {
                drivingGamepad.runRumbleEffect(endgameRumble);
                utilityGamepad.runRumbleEffect(endgameRumble);
            }

            driveModeButton.updateButton(drivingGamepad.x);
            driveModeButton.shortPress();
            driveModeButton.longPress();

            driveSystem.setSpeed(liftSystem.getRobotSpeed());
            driveSystem.setFieldCentricMode(driveModeButton.getLongToggle());

            driveSystem.run();
            //driveSystem.showInfo(telemetry);

            liftSystem.run();
            liftSystem.resetLift(true);
            liftSystem.showInfo(telemetry);

            if (liftSystem.isStackMode())
                liftNewClaw.setAxlePos(LiftSystem.LiftPositions.STARTING_POS.axlePos);
            else
                liftNewClaw.setAxlePos(liftSystem.getAxleTarget());
            liftNewClaw.run();
            //liftNewClaw.showInfo(telemetry);

            //loopTime.measure();

            telemetry.addData("OpMode Seconds: ", opModeTimer.seconds());
            telemetry.update();
        }
    }
}
