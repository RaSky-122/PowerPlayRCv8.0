package org.firstinspires.ftc.teamcode.rasky;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.rasky.components.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.rasky.utilities.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;
import org.firstinspires.ftc.teamcode.rasky.utilities.DrivingMotors;
import org.firstinspires.ftc.teamcode.rasky.components.RobotCentricDrive;
import org.firstinspires.ftc.teamcode.rasky.utilities.Gyroscope;
import org.firstinspires.ftc.teamcode.rasky.components.LiftControl;

/**
 * The main TeleOP for the driving period of the game.
 *
 * @author Lucian
 * @version 1.2
 */
@TeleOp(name = "Main Driving", group = Constants.mainGroup)
public class MainDrivingOp extends LinearOpMode {

    DrivingMotors motors;
    Gyroscope gyroscope;

    RobotCentricDrive robotCentricDrive;
    FieldCentricDrive fieldCentricDrive;
    Gamepad drivingGamepad;


    @Override
    public void runOpMode() throws InterruptedException {

        motors = new DrivingMotors(hardwareMap);
        gyroscope = new Gyroscope(hardwareMap);

        //Set the driving gamepad to the desired gamepad
        drivingGamepad = gamepad1;

        gyroscope.Init();
        motors.Init(false, true);
        robotCentricDrive = new RobotCentricDrive(motors, drivingGamepad);
        fieldCentricDrive = new FieldCentricDrive(motors, drivingGamepad, gyroscope);

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        Button driveModeButton = new Button();
        Button liftButton = new Button();

        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {
            driveModeButton.updateButton(drivingGamepad.x);
            driveModeButton.shortPress();
            driveModeButton.longPress();

            liftButton.updateButton(drivingGamepad.y);
            liftButton.shortPress();
            liftButton.longPress();

            robotCentricDrive.setReverse(driveModeButton.getShortToggle());



            telemetry.update();
        }
    }
}