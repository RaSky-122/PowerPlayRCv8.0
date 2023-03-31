package org.firstinspires.ftc.teamcode.rasky.components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedServo;

public class LiftNewClaw {

    HardwareMap hardwareMap;
    Gamepad gamepad;

    WrappedServo servoClaw;
    WrappedServo servoJoint;
    //adaugat de raca , nu te speria
    WrappedServo servoAxle;

    public LiftNewClaw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad;
    }

    public enum ClawModes {
        CLOSED(0), OPEN(0.55);

        public double position = 0;

        ClawModes(double value) {
            this.position = value;
        }
    }

    public enum JointModes {
        FRONT(0.125), BACK(0.725);

        public double position = 0;

        JointModes(double value) {
            this.position = value;
        }
    }

    MotionState beginning = new MotionState(0, 0, 0);
    MotionState end = new MotionState(200, 0, 0);

    MotionProfile profileBackward = MotionProfileGenerator.generateSimpleMotionProfile(
            beginning, end, 10, 2, 50
    );

    MotionProfile profileForward = MotionProfileGenerator.generateSimpleMotionProfile(
            end, beginning, 10, 2, 50
    );

    ClawModes clawState = ClawModes.OPEN;

    JointModes jointState = JointModes.FRONT;

    //adaugat de raca,nu te speria

    VoltageSensor voltageSensor;

    public void Init() {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        servoClaw = new WrappedServo(hardwareMap);
        servoClaw.Init("claw", false, true);
        servoClaw.setPWMRange(500, 2500);
        servoClaw.setPosition(clawState.position);

        servoJoint = new WrappedServo(hardwareMap);
        servoJoint.Init("joint", false, false);
        servoJoint.setPWMRange(500, 2500);
        servoJoint.setPosition(jointState.position);

//astea 4 erau comentate

        servoAxle = new WrappedServo(hardwareMap);
        servoAxle.Init("axle", false, true);
        servoAxle.setPWMRange(500, 2500);

//      motorAxle = hardwareMap.get(DcMotorEx.class, "motorAxle");
//      motorAxle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      motorAxle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      motorAxle.setDirection(DcMotorSimple.Direction.REVERSE);
//      motorAxle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    Button clawButton = new Button();
    MotionProfile profileCurrent = MotionProfileGenerator.generateSimpleMotionProfile(
            beginning, beginning, 20, 5, 10
    );

    ElapsedTime axleProfileTimer = new ElapsedTime();
    boolean profileInit = false;

    public void run() {

        clawButton.updateButton(gamepad.x);

        if (clawButton.toggle()) {
            axleProfileTimer.reset();
            profileInit = true;
        }

        if (clawButton.getToggleStatus()) {
            clawState = ClawModes.CLOSED;

        } else {
            clawState = ClawModes.OPEN;
        }

        //if (motorAxle.getCurrentPosition() >= 100)
        if (servoAxle.getPosition() <= 0.3)
            jointState = JointModes.BACK;
        else
            jointState = JointModes.FRONT;

        servoClaw.setPosition(clawState.position);
        servoJoint.setPosition(jointState.position);
    }

    public double AxlePos() {
        //si aici am scos comentariu
        return servoAxle.getPosition();
    }

    public void setAxlePos(double pos) {
        servoAxle.setPosition(pos);
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Claw State: ", clawState);
        telemetry.addData("Claw Position: ", servoClaw.getPosition());

        telemetry.addData("Joint State: ", jointState);
        telemetry.addData("Joint Position: ", servoJoint.getPosition());

        telemetry.addData("Axle Position: ", servoAxle.getPosition());


        telemetry.addData("Button Toggle: ", clawButton.getToggleStatus());


    }

}
