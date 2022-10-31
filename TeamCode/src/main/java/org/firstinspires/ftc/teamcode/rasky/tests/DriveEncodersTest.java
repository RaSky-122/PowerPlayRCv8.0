package org.firstinspires.ftc.teamcode.rasky.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.rasky.components.DebugDriveMode;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;
import org.firstinspires.ftc.teamcode.rasky.utilities.DrivingMotors;
import org.firstinspires.ftc.teamcode.rasky.components.RobotCentricDrive;


/**
 * <p>
 * Class meant for testing the encoders on the drive wheels.
 * <p>
 * It gets and shows the encoder position of each motor and has a debug test for individual testing.
 *
 * @author Lucian
 * @version 1.1
 */
@TeleOp(name = "DriveEncoders Test", group = Constants.testGroup)
public class DriveEncodersTest extends LinearOpMode {

    DrivingMotors motors;
    DebugDriveMode debugDriveMode;
    RobotCentricDrive robotCentricDrive;
    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = gamepad1;

        motors = new DrivingMotors(hardwareMap);
        motors.Init();

        debugDriveMode = new DebugDriveMode(motors, gamepad);
        robotCentricDrive = new RobotCentricDrive(motors, gamepad);

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.addLine("Instructions:");
            telemetry.addLine("While running the test, press RIGHT_BUMPER to enter DebugDriveMode " +
                    "or LEFT_BUMPER to exit DebugDriveMode");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        boolean debugMode = false;
        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {

            if (gamepad.right_bumper) {
                debugMode = true;
            }
            if (gamepad.left_bumper) {
                debugMode = false;
            }

            if (debugMode) {
                debugDriveMode.run();
            } else {
                robotCentricDrive.run();
            }

            telemetry.addData("Is Debug Mode On? : ", debugMode);
            telemetry.addData("LeftRear Encoder: ", motors.leftRear.getCurrentPosition());
            telemetry.addData("RightRear Encoder: ", motors.rightRear.getCurrentPosition());
            telemetry.addData("LeftFront Encoder: ", motors.leftFront.getCurrentPosition());
            telemetry.addData("RightFront Encoder: ", motors.rightFront.getCurrentPosition());
            telemetry.update();
        }
    }
}
