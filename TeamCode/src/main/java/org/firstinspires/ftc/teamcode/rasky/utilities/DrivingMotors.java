package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * <p>
 * A class that handles all of the 4 driving motors.
 * <p>
 * It initializes all of the motors given a set of parameters and then they are ready to use
 * through the object.
 * <p>
 * !! CALL INIT() METHOD BEFORE USING !!
 * @author Lucian
 * @version 1.4
 */
public class DrivingMotors {
    public DcMotorEx leftRear, rightRear, leftFront, rightFront;
    HardwareMap hardwareMap;

    public DrivingMotors(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Call before using the class.
     * This method initializes each driving motor.
     */
    public void Init() {
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        ArrayList<DcMotorEx> motors = new ArrayList<DcMotorEx>(Arrays.asList(leftRear, rightRear, leftFront, rightFront));

        //This initializes the motors with the given settings
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //Reverse any motors if needed here:
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
