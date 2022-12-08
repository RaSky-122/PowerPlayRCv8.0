package org.firstinspires.ftc.teamcode.rasky.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public WrappedMotor2 liftMotor;

    double tolerance = 10;
    double speed = 1;

    public LiftSystem(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    double kP = 0.005, kI = 0, kD = 0.0001;

    /**
     * Call this method before using the object.
     */
    public void Init() {
        liftMotor = new WrappedMotor2(hardwareMap);
        liftMotor.Init("liftMotor", true, false, true, true);

        liftMotor.setPositionPIDMode(true);
        liftMotor.setPositionPID(kP, kI, kD);

        liftMotor.setTolerance(tolerance);
        liftMotor.setSpeedMultiplier(speed);
        liftMotor.setEncoderDirection(1);
        liftMotor.holdMode(true);
    }

    //Lift positions in encoder ticks
    public enum LiftPositions {
        HIGH_JUNCTION(1460, 0.5),
        MEDIUM_JUNCTION(1000, 0.6),
        LOW_JUNCTION(600, 0.7),
        GROUND_JUNCTION(70, 0.85),
        STARTING_POS(5, 0.85);

        public double position = 0;
        public double speed = 0;

        LiftPositions(double value, double speed) {
            this.position = value;
            this.speed = speed;
        }
    }

    Button resetButton = new Button();
    Button liftUpButton = new Button();
    Button liftDownButton = new Button();
    Button manualButton = new Button();
    LiftPositions state = LiftPositions.STARTING_POS;
    int toggleStates = 0;
    boolean manualMode = false;

    public void run() {
        resetButton.updateButton(gamepad.b);
        liftUpButton.updateButton(gamepad.y);
        liftDownButton.updateButton(gamepad.a);
        manualButton.updateButton(gamepad.left_bumper);

        if (resetButton.press())
            toggleStates = 0;

        if (manualButton.toggle()) {
            if (!manualMode) {
                manualMode = true;
                liftMotor.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (manualMode) {
                manualMode = false;
                liftMotor.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if (liftUpButton.toggle() && toggleStates < 4 && !manualMode) {
            toggleStates++;
        }

        if (liftDownButton.toggle() && toggleStates > 0 && !manualMode) {
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

        if (manualMode && liftDownButton.press())
            liftMotor.setPower(-0.5);
        else if (manualMode && liftUpButton.press())
            liftMotor.setPower(0.6);
        else if (manualMode)
            liftMotor.setPower(0.1);

        if (!manualMode) {
            liftMotor.setTargetPosition(state.position);
            liftMotor.updatePosition();
        }

    }

    public double getRobotSpeed() {
        return state.speed;
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Lift NrState: ", toggleStates);
        telemetry.addData("Lift State: ", state);
        telemetry.addData("Manual Mode: ", manualMode);
        telemetry.addData("Motor Mode: ", liftMotor.motor.getMode());

        telemetry.addData("Lift TargetPos: ", liftMotor.targetPosition);
        telemetry.addData("Lift Current Position: ", liftMotor.currentPosition);

        telemetry.addData("Motor Power: ", liftMotor.motor.getPower());
        telemetry.addData("Lift Encoder: ", liftMotor.motor.getCurrentPosition());
        telemetry.addData("Is Busy: ", liftMotor.motor.isBusy());
        telemetry.addData("Lift PIDF: ", liftMotor.motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).toString());

        telemetry.addData("Position Tolerance: ", tolerance);
        telemetry.addData("Motor Direction: ", liftMotor.direction);
    }

}
