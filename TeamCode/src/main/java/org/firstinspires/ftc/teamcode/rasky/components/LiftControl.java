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

    LiftControl(HardwareMap hardwareMap, int tolerance)
    {
        this.tolerance = tolerance;
        this.target = 0;

        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    void setTarget(int val) {
        target = val;
    }

    void run(){



    }

}
