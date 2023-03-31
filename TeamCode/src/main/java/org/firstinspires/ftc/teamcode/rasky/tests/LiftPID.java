package org.firstinspires.ftc.teamcode.rasky.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class LiftPID extends OpMode {

    DcMotorEx liftMotor;
    DcMotorEx liftMotor2;

    PIDFController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(new PIDCoefficients(p, i, d));
    }

    @Override
    public void loop() {
        controller.setTargetPosition(target);
        int currPos = liftMotor.getCurrentPosition();
        double pidVal = controller.update(currPos);

        double ff = 1 * f;
        double power = ff + pidVal;

        liftMotor.setPower(power);
        liftMotor2.setPower(power);

        telemetry.addData("target: ", target);
        telemetry.addData("power: ", power);
        telemetry.addData("pos: ", currPos);

        telemetry.update();
    }
}
