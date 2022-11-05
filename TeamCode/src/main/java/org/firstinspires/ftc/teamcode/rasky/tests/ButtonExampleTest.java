package org.firstinspires.ftc.teamcode.rasky.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;

/**
 * <p>
 * This class is meant as a demonstration / test of the Button.java class.
 * <p>
 * It uses 3 buttons, one normal press, one toggle and one time based (short / long).
 *
 * @author Lucian
 * @version 1.2
 */
@TeleOp(name = "Button Example Test", group = Constants.testGroup)
public class ButtonExampleTest extends LinearOpMode {

    Button normalPress = new Button();
    Button timeBasedPress = new Button();
    Button toggleBasedPress = new Button();

    Gamepad gamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = gamepad1;

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.addLine("Instructions:");
            telemetry.addLine("X/Square is normal press, " +
                    "Y/Triangle is a toggle, " +
                    "B/Circle is short press and long press");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;


        int changedStatus = 0;
        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {
            normalPress.updateButton(gamepad.x);
            toggleBasedPress.updateButton(gamepad.y);
            timeBasedPress.updateButton(gamepad.b);

            if (timeBasedPress.longPress() || timeBasedPress.shortPress() || toggleBasedPress.toggle())
                changedStatus++;


            telemetry.addData("Normal Press Status: ", normalPress.press());
            telemetry.addData("Toggle Press Status: ", toggleBasedPress.toggle());
            telemetry.addData("Toggled Status: ", toggleBasedPress.getToggleStatus());
            telemetry.addData("Short Press Status: ", timeBasedPress.shortPress());
            telemetry.addData("Short Press Toggle Status: ", timeBasedPress.getShortToggle());
            telemetry.addData("Long Press Status: ", timeBasedPress.longPress());
            telemetry.addData("Long Press Toggle Status: ", timeBasedPress.getLongToggle());
            telemetry.addData("Statuses Changed: ", changedStatus);

            telemetry.update();
        }

    }
}
