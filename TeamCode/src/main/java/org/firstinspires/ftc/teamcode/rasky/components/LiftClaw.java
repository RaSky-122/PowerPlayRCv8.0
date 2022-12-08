package org.firstinspires.ftc.teamcode.rasky.components;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.Button;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedServo;

/**
 * Lift claw mechanism.
 * <p>
 * !! CALL INIT() METHOD BEFORE USING !!
 *
 * @author Lucian
 * @version 1.1
 */
public class LiftClaw {
    WrappedServo clawServo;
    HardwareMap hardwareMap;
    Gamepad gamepad;

    public LiftClaw(HardwareMap hardwareMap, Gamepad gamepad) {
        this.hardwareMap = hardwareMap;
        this.gamepad = gamepad;
    }

    /**
     * Call this method before using the claw.
     */
    public void Init() {
        clawServo = new WrappedServo(hardwareMap);
        clawServo.Init("claw", false, true);
        clawServo.setPWMRange(500, 2500);
    }

    public enum ClawModes {
        CLOSED(0),
        OPEN(0.5 );

        public double position = 0;

        ClawModes(double value) {
            this.position = value;
        }
    }

    Button clawButton = new Button();
    ClawModes clawState = ClawModes.OPEN;

    /**
     * Asynchronous function that updates the claw logic.
     */
    public void run() {
        clawButton.updateButton(gamepad.x);
        clawButton.toggle();

        if (clawButton.getToggleStatus())
            clawState = ClawModes.CLOSED;
        else
            clawState = ClawModes.OPEN;

        clawServo.setPosition(clawState.position);

    }

    public void showInfo(Telemetry telemetry) {
        telemetry.addData("Servo State: ", clawState);
    }
}
