package org.firstinspires.ftc.teamcode.rasky.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftControl {

    DcMotorEx lift;
    int tolerance;
    int target;
    double speed;

    public LiftControl(HardwareMap hardwareMap, int tolerance, double speed)
    {
        this.tolerance = tolerance;

        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);

        this.target = lift.getCurrentPosition();
    }

    void setTarget(int val) {
        target = val;
    }

    void setSpeed(double val) {
        speed = val;
    }

    void run(){

        double dir; // direction

        if(lift.getCurrentPosition() < target)
            dir = 1;
        else
            dir = -1;

        if(lift.getCurrentPosition() >= target - tolerance && lift.getCurrentPosition() <= target + tolerance)
            lift.setPower(speed * dir);
        else
            lift.setPower(0);
    }
}
