package org.firstinspires.ftc.teamcode.rasky.tests;

import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.Constants;

/**
 * Class meant for testing a servo motion profile.
 *
 * @author Lucian
 * @version 1.0
 */
@TeleOp(name = "Servo Motion Profile", group = Constants.testGroup)
public class TestServoMotionProfile extends LinearOpMode {

    Gamepad gamepad;
    ServoImplEx servo;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = gamepad1;
        servo = hardwareMap.get(ServoImplEx.class, "servo");
        servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        //This while loop will run after initialization until the program starts or until stop
        //is pressed
        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("Initialization Ready");
            telemetry.addLine("Instructions:");
            telemetry.addLine("Press X on the gamepad to switch between 0 and 1" +
                    " in servo positions.");
            telemetry.update();
        }

        //This catches the stop button before the program starts
        if (isStopRequested()) return;

        Button moveServo = new Button();

        MotionState beginning = new MotionState(0.83, 0, 0);
        servo.setPosition(beginning.getX());
        MotionState end = new MotionState(0, 0, 0);

        MotionProfile profile;

        ElapsedTime elapsedTime = new ElapsedTime();
        boolean last = false, init = false;
        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {
            last = moveServo.getToggleStatus();
            moveServo.updateButton(gamepad.x);

            if (moveServo.toggle()) {
                elapsedTime.reset();
                init = true;
            }

            if (moveServo.getToggleStatus()) {
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        beginning,
                        end,
                        20,
                        5,
                        10
                );
            } else {
                profile = MotionProfileGenerator.generateSimpleMotionProfile(
                        end,
                        beginning,
                        2,
                        0.5,
                        5
                );
            }


            if (init)
                servo.setPosition(profile.get(elapsedTime.seconds()).getX());

            telemetry.addData("Servo Target Pos: ", servo.getPosition());
            telemetry.update();
        }

    }
}
