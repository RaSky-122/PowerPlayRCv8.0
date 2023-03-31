package org.firstinspires.ftc.teamcode.rasky.tests;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
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
@TeleOp(name = "Motor PID Test", group = Constants.testGroup)
public class MotorVelPIDTest extends LinearOpMode {

    Gamepad gamepad;
    ServoImplEx servo;
    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepad = gamepad1;
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        Button setPos = new Button();
        double targetPos = 0;

        PIDCoefficients coeffs = new PIDCoefficients(8, 3, 0);
        PIDFController controller = new PIDFController(coeffs, 0, 0, 0, (x, v) -> 0.1);

        double position;
        double pow;

        //Main while loop that runs during the match
        while (opModeIsActive() && !isStopRequested()) {
            setPos.updateButton(gamepad.x);

            if (setPos.getToggleStatus())
                targetPos = 2000;
            else
                targetPos = 0;

            controller.setTargetPosition(targetPos);

            position = motor.getCurrentPosition();
            pow = controller.update(position);

            //controller.setTargetVelocity();

            motor.setPower(pow);

        }

    }
}
