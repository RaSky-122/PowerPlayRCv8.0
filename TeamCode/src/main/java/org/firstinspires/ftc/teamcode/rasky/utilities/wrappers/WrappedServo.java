package org.firstinspires.ftc.teamcode.rasky.utilities.wrappers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * A wrapper class for servos that makes it easier to use them.
 *
 * @author Lucian
 * @version 1.2
 */
public class WrappedServo {
    public ServoImplEx normalServo;
    public CRServoImplEx continuousServo;
    HardwareMap hardwareMap;

    boolean continuousMode = false;
    boolean isReversed = false;

    public WrappedServo(HardwareMap hardwareMap, double minPWM, double maxPWM) {
        this.hardwareMap = hardwareMap;
        normalServo.setPwmRange(new PwmControl.PwmRange(minPWM, maxPWM));
        continuousServo.setPwmRange(new PwmControl.PwmRange(minPWM, maxPWM));
    }

    /**
     * Call this function before using the object.
     *
     * @param name           The name of the servo
     * @param continuousMode If the servo is of type CRServo
     * @param isReversed     If the servo is reversed or not
     */
    public void Init(String name, boolean continuousMode, boolean isReversed) {
        this.continuousMode = continuousMode;
        if (continuousMode)
            continuousServo = hardwareMap.get(CRServoImplEx.class, name);
        else
            normalServo = hardwareMap.get(ServoImplEx.class, name);

        setReversed(isReversed);
    }

    public void setPosition(double position) {
        if (continuousMode) {
            position *= 2;
            position--;

            continuousServo.setPower(position);
        } else
            normalServo.setPosition(position);
    }

    public void setPower(double power) {
        if (continuousMode)
            continuousServo.setPower(power);
        else {
            power++;
            power = power / 2.0;

            normalServo.setPosition(power);
        }
    }

    public void setReversed(boolean isReversed) {
        this.isReversed = isReversed;
    }
}
