package org.firstinspires.ftc.teamcode.rasky.components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedMotor;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedMotor2;

/**
 * The class that operates the lift.
 *
 * <p>
 * !! CALL INIT() METHOD BEFORE USING !!
 *
 * @author Lucian
 * @version 1.3
 */
public class LiftSystem {
    Gamepad gamepad;
    HardwareMap hardwareMap;
    public WrappedMotor liftMotor;

    double tolerance = 15;
    double speed = 0.8;

    public LiftSystem(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    double kP = 0.005, kI = 0, kD = 0.0001;

    /**
     * Call this method before using the object.
     */
    public void Init() {
        liftMotor = new WrappedMotor(hardwareMap);
        liftMotor.Init("liftMotor", "liftMotor2", false, false, true, false);

        liftMotor.setPositionPIDMode(true);
        liftMotor.setPositionPID(kP, kI, kD);

        liftMotor.setTolerance(tolerance);
        liftMotor.setSpeedMultiplier(speed);

        liftMotor.setEncoderDirection(1);
        liftMotor.holdMode(true);
    }

    //Lift positions in encoder ticks
    public enum LiftPositions {
        HIGH_JUNCTION(1585, 0.6, 0),
        MEDIUM_JUNCTION(915, 0.7, 0),
        LOW_JUNCTION(125, 0.8, 0.49),
        GROUND_JUNCTION(0, 0.85, 0.77),
        STARTING_POS(0, 0.85, 0.82);

        public double position = 0;
        public double speed = 0;
        public double axlePos = 0;

        LiftPositions(double value, double speed, double axlePos) {
            this.position = value;
            this.speed = speed;
            this.axlePos = axlePos;
        }
    }

    Button resetButton = new Button();
    Button liftUpButton = new Button();
    Button liftDownButton = new Button();
    Button manualButton = new Button();
    Button stackButton = new Button();
    double stackHeight = 380;
    double oldStackHeight = 380;

    LiftPositions state = LiftPositions.STARTING_POS;
    int toggleStates = 0;
    boolean manualMode = false;
    boolean stackMode = false;

    public void run() {
        resetButton.updateButton(gamepad.b);
        liftUpButton.updateButton(gamepad.y);
        liftDownButton.updateButton(gamepad.a);
        manualButton.updateButton(gamepad.left_bumper);
        stackButton.updateButton(gamepad.right_bumper);

        if (manualButton.toggle()) {
            if (!manualMode) {
                stackMode = false;
                manualMode = true;
            } else if (manualMode) {
                manualMode = false;
                stackMode = false;
                liftMotor.resetPID();
            }
        }

        if (stackButton.toggle()) {
            if (stackMode == false) {
                stackMode = true;
                stackHeight = oldStackHeight;
            } else {
                stackMode = false;
                toggleStates = 2;
            }
            manualMode = false;
        }

        if (liftUpButton.toggle() && !manualMode) {
            if (stackMode) {
                if (stackHeight < 380)
                    stackHeight += 80;
            } else if (toggleStates < 4)
                toggleStates++;
        }

        if (liftDownButton.toggle() && !manualMode) {
            if (stackMode) {
                if (stackHeight > 60)
                    stackHeight -= 80;
            } else if (toggleStates > 0)
                toggleStates--;
        }

        switch (toggleStates) {
            case 1:
                state = LiftPositions.GROUND_JUNCTION;
                break;
            case 2:
                state = LiftPositions.LOW_JUNCTION;
                break;
            case 3:
                state = LiftPositions.MEDIUM_JUNCTION;
                break;
            case 4:
                state = LiftPositions.HIGH_JUNCTION;
                break;
            default:
                state = LiftPositions.STARTING_POS;
                break;
        }

        if (manualMode) {
            if (liftDownButton.press())
                liftMotor.setPower(-0.7);
            else if (liftUpButton.press())
                liftMotor.setPower(0.9);
            else if (manualMode)
                liftMotor.setPower(0.15);

            liftMotor.setTargetPosition(liftMotor.motor.getCurrentPosition());
        }


        if (!manualMode) {
            if (!stackMode)
                liftMotor.setTargetPosition(state.position);
            else
                liftMotor.setTargetPosition(stackHeight);
            liftMotor.updatePosition();
        }
    }

    public double getAxleTarget() {
        return state.axlePos;
    }

    public void resetLift(boolean able) {
        if (resetButton.press() && able) {
            toggleStates = 0;
            if (stackMode && stackHeight < 850) {
                oldStackHeight = stackHeight;
                stackHeight = 850;
            }
        }
    }

    public boolean isStackMode() {
        return stackMode;
    }

    public double getRobotSpeed() {
        return state.speed;
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Lift NrState: ", toggleStates);
        telemetry.addData("Lift State: ", state);
        telemetry.addData("Manual Mode: ", manualMode);
        telemetry.addData("Stack Mode: ", stackMode);
        telemetry.addData("Stack Height: ", stackHeight);
        telemetry.addData("Motor Mode: ", liftMotor.motor.getMode());

        telemetry.addData("Lift TargetPos: ", liftMotor.targetPosition);
        telemetry.addData("Lift Current Position: ", liftMotor.currentPosition);

        telemetry.addData("Motor Power: ", liftMotor.motor.getPower());
        telemetry.addData("Motor Direction: ", liftMotor.motor.getDirection());
        telemetry.addData("Lift Encoder: ", liftMotor.motor.getCurrentPosition());
        telemetry.addData("Motor Vel: ", liftMotor.motor.getVelocity());
        telemetry.addData("PID Pos: ", liftMotor.calcPos);
        telemetry.addData("PID Pow: ", liftMotor.power);

        telemetry.addData("Position Tolerance: ", tolerance);
        telemetry.addData("Motor Direction: ", liftMotor.direction);
    }

}
