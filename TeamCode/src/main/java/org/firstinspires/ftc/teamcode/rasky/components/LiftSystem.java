package org.firstinspires.ftc.teamcode.rasky.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedMotor;

/**
 * The class that operates the lift.
 *
 * <p>
 * !! CALL INIT() METHOD BEFORE USING !!
 *
 * @author Lucian
 * @version 1.2
 */
public class LiftSystem {
    Gamepad gamepad;
    HardwareMap hardwareMap;
    public WrappedMotor liftMotor;

    double tolerance = 10;
    double speed = 1;

    public LiftSystem(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.hardwareMap = hardwareMap;
    }

    double kP = 0.0075, kI = 0, kD = 0;
    /**
     * Call this method before using the object.
     */
    public void Init() {
        liftMotor = new WrappedMotor(hardwareMap);
        liftMotor.Init("liftMotor", true, false, true);

        liftMotor.setPositionPIDMode(true);
        liftMotor.setPositionPID(kP, kI, kD);

        liftMotor.setTolerance(tolerance);
        liftMotor.setSpeed(speed);
        liftMotor.setEncoderDirection(1);
        liftMotor.holdMode(true);
    }

    //Lift positions in encoder ticks
    enum LiftPositions {
        HIGH_JUNCTION(1415),
        MEDIUM_JUNCTION(975),
        LOW_JUNCTION(575),
        GROUND_JUNCTION(55),
        STARTING_POS(0);

        double position = 0;

        LiftPositions(double value) {
            this.position = value;
        }
    }

    Button resetButton = new Button();
    Button liftButton = new Button();
    LiftPositions state;
    int toggleStates = 0;

    public void run() {
        resetButton.updateButton(gamepad.b);
        liftButton.updateButton(gamepad.y);

        if (resetButton.press())
            toggleStates = 0;


        if (liftButton.toggle())
            toggleStates++;

        switch (toggleStates % 5) {
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

        liftMotor.setTargetPosition(state.position);
        liftMotor.updatePosition();
    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Lift NrState: ", toggleStates);
        telemetry.addData("Lift State: ", state);

        telemetry.addData("Lift TargetPos: ", liftMotor.targetPosition);
        telemetry.addData("Lift Current Position: ", liftMotor.currentPosition);

        telemetry.addData("Motor Power: ", liftMotor.motor.getPower());
        telemetry.addData("Lift Encoder: ", liftMotor.motor.getCurrentPosition());

        telemetry.addData("Position Tolerance: ", tolerance);
        telemetry.addData("Motor Direction: ", liftMotor.direction);
    }

}
