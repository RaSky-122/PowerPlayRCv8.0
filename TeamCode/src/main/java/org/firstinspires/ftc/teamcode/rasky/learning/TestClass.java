package org.firstinspires.ftc.teamcode.rasky.learning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class TestClass extends LinearOpMode {

    DcMotorEx motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPositionTolerance(10);
        motor.setTargetPosition(50);
        motor.setPower(1);
    }

}
