package org.firstinspires.ftc.teamcode.rasky.utilities;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import static org.firstinspires.ftc.teamcode.rasky.utilities.UsefulMethods.*;

/**
 * Constants class, compatible with dashboard
 *
 * @author Lucian
 * @version 1.2
 */
@Config
public class Constants {
    // TeleOP string constants for group names
    public static final String testGroup = "test";
    public static final String mainGroup = "main";

    public static final double lateralError = 0.11;

    //Deadwheels positions for RoadRunner used in StandardTrackingWheelLocalizer.java
    public static final Pose2d leftEncoderPos = new Pose2d(cmToInch(5.15), cmToInch(-6.6 + lateralError/2), 0);
    public static final Pose2d rightEncoderPos = new Pose2d(cmToInch(5.15), cmToInch(7.65 - lateralError/2), 0);
    public static final Pose2d middleEncoderPos = new Pose2d(cmToInch(-11.5), cmToInch(0.7), Math.toRadians(90));
}
