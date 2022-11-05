package org.firstinspires.ftc.teamcode.rasky.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;

/**
 * Class meant for testing a servo in the normal (positioning) mode.
 *
 * @author Lucian
 * @version 1.1
 */
@TeleOp(name = "Normal Servo Test", group = Constants.testGroup)
public class NormalServoTest extends LinearOpMode {
    Servo servo;
    Gamepad gamepad;

    enum ServoStates {
        LEFT(0),
        MIDDLE(0.5),
        RIGHT(1);

        double value = 0;

        ServoStates(double value) {
            this.value = value;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.servo.get("servo");
        gamepad = gamepad1;

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.addLine("Instructions: ");
            telemetry.addLine("Press DPad Left for the left most position, " +
                    "Press DPad Up for the middle position, " +
                    "Press DPad Right for the right most position");
            telemetry.addLine("WARNING: THE SERVO WILL MOVE ON START");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        Button leftPos = new Button();
        Button midPos = new Button();
        Button rightPos = new Button();
        ServoStates state = ServoStates.MIDDLE;
        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {
            leftPos.updateButton(gamepad.dpad_left);
            midPos.updateButton(gamepad.dpad_up);
            rightPos.updateButton(gamepad.dpad_right);

            if (midPos.press()) {
                state = ServoStates.MIDDLE;
            } else if (leftPos.press()) {
                state = ServoStates.LEFT;
            } else if (rightPos.press()) {
                state = ServoStates.RIGHT;
            }

            servo.setPosition(state.value);

            telemetry.addData("Servo Position: ", state.name());
            telemetry.addData("Servo True Position: ", servo.getPosition());
            telemetry.addData("Servo Direction: ", servo.getDirection());
            telemetry.update();
        }
    }
}
