package org.firstinspires.ftc.teamcode.rasky.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class TestClass extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor1");


        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("Init Ready!");
            telemetry.update();
        }

        if (isStopRequested())
            return;

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addLine("running...");

            telemetry.update();
        }

    }
}