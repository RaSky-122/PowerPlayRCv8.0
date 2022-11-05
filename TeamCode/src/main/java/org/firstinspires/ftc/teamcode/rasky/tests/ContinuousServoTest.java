package org.firstinspires.ftc.teamcode.rasky.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;

/**
 * Class for testing Continuous Rotation Servos.
 *
 * @author Lucian
 * @version 1.0
 */
@TeleOp(name = "Continuous Servo Test", group = Constants.testGroup)
public class ContinuousServoTest extends LinearOpMode {

    Gamepad gamepad;
    CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = gamepad1;
        servo = hardwareMap.crservo.get("servo");

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.addLine("Instructions:");
            telemetry.addLine("Use DPad Up/Down to Increment/Decrement Speed, " +
                    "Hold X/Square to power the servo");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        double power = 0;
        Button incrementSpeed = new Button();
        Button decrementSpeed = new Button();
        Button enableServo = new Button();
        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {

            incrementSpeed.updateButton(gamepad.dpad_up);
            decrementSpeed.updateButton(gamepad.dpad_down);
            enableServo.updateButton(gamepad.x);

            if (incrementSpeed.toggle()) {
                power += 0.1;
            }
            if (decrementSpeed.toggle()) {
                power -= 0.1;
            }
            if (enableServo.press()) {
                servo.setPower(power);
            } else {
                servo.setPower(0);
            }

            telemetry.addData("Is Servo Powered?: ", (servo.getPower() != 0));
            telemetry.addData("Theoretical Power: ", power);
            telemetry.addData("Actual Power: ", servo.getPower());
            telemetry.update();
        }

    }
}
