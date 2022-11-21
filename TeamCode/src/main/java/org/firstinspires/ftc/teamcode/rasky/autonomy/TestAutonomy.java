package org.firstinspires.ftc.teamcode.rasky.autonomy;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rasky.components.LiftSystem;
import org.firstinspires.ftc.teamcode.rasky.detection.AutonomyDetection;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedMotor;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;

/**
 * Visually see and test Road Runner trajectories using MeepMeep.
 *
 * @author Lucian
 * @version 1.0
 */
@Autonomous(name = "Test Autonomy", group = "tests")
public class TestAutonomy extends LinearOpMode {

    //AutonomyDetection detectionSystem;
    WrappedMotor liftMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setPoseEstimate(new Pose2d(35.5, -62, Math.toRadians(90)));
        //detectionSystem = new AutonomyDetection(hardwareMap);
        //detectionSystem.init();

        liftMotor = new WrappedMotor(hardwareMap);
        liftMotor.Init("liftMotor", true, false, true, true);

        double kP = 0.0075, kI = 0.002, kD = 0;
        double tolerance = 10;
        double speed = 1;

        liftMotor.setPositionPIDMode(true);
        liftMotor.setPositionPID(kP, kI, kD);

        liftMotor.setTolerance(tolerance);
        liftMotor.setSpeedMultiplier(speed);
        liftMotor.setEncoderDirection(1);
        liftMotor.holdMode(true);

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(55)
                .back(3)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(startTraj.end())
                .forward(9)
                .lineToLinearHeading(new Pose2d(startTraj.end().getX(), startTraj.end().getY() - 1.75, Math.toRadians(-90)))
                .forward(23)
                .lineToLinearHeading(startTraj.end())
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            /*AprilTagDetection detectedTag = detectionSystem.detect();
            if (detectedTag != null) {
                telemetry.addData("Detected Tag ID: " + detectedTag.id, " ");
            }*/

            telemetry.update();
        }

        drive.followTrajectorySequence(startTraj);

        drive.followTrajectorySequenceAsync(cycleTrajectory);
        while (drive.isBusy())
        {
            liftMotor.setTargetPosition(150);
            liftMotor.updatePosition();
            drive.update();
        }

        return;
    }
}
