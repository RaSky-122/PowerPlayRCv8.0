package org.firstinspires.ftc.teamcode.rasky.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.rasky.utilities.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;
import org.firstinspires.ftc.teamcode.rasky.utilities.Gyroscope;

/**
 * Class for testing the built-in Control Hub Gyroscope.
 *
 * @author Lucian
 * @version 1.0
 */
@TeleOp(name = "Gyroscope Test", group = Constants.testGroup)
public class GyroscopeTest extends LinearOpMode {

    Gyroscope gyroscope;

    @Override
    public void runOpMode() throws InterruptedException {

        gyroscope = new Gyroscope(hardwareMap);
        gyroscope.Init();

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.addLine("Instructions:");
            telemetry.addLine("This test has no built-in controls, just move the " +
                    "robot in the field and see the actions it has in telemetry");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {

            gyroscope.showAllInfo(telemetry);
            telemetry.update();
        }

    }
}
