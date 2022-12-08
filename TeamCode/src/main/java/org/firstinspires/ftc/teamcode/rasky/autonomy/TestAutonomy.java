package org.firstinspires.ftc.teamcode.rasky.autonomy;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rasky.components.LiftClaw;
import org.firstinspires.ftc.teamcode.rasky.components.LiftSystem;
import org.firstinspires.ftc.teamcode.rasky.detection.AutonomyDetection;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedMotor;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedMotor2;
import org.firstinspires.ftc.teamcode.rasky.utilities.wrappers.WrappedServo;
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants;
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

    AutonomyDetection detectionSystem;
    WrappedMotor2 liftMotor;
    WrappedServo clawServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));

        detectionSystem = new AutonomyDetection(hardwareMap);
        detectionSystem.init();

        liftMotor = new WrappedMotor2(hardwareMap);
        liftMotor.Init("liftMotor", true, false, true, true);

        double kP = 0.005, kI = 0, kD = 0.0001;
        double tolerance = 10;
        double speed = 1;

        liftMotor.setPositionPIDMode(true);
        liftMotor.setPositionPID(kP, kI, kD);

        liftMotor.setTolerance(tolerance);
        liftMotor.setSpeedMultiplier(speed);
        liftMotor.setEncoderDirection(1);
        liftMotor.holdMode(true);

        clawServo = new WrappedServo(hardwareMap);
        clawServo.Init("claw", false, true);
        clawServo.setPWMRange(500, 2500);
        clawServo.setPosition(LiftClaw.ClawModes.CLOSED.position);

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(54.25)
                .back(3)
                .turn(Math.toRadians(45))
                .build();

        TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint angCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL * 1.5, DriveConstants.TRACK_WIDTH);

        TrajectorySequence scoreTraj = drive.trajectorySequenceBuilder(startTraj.end())
                .addTemporalMarker(() -> {
                    liftMotor.setTargetPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.position);
                })
                .setVelConstraint(velCon)
                .forward(9.5)
                .resetVelConstraint()
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    clawServo.setPosition(LiftClaw.ClawModes.OPEN.position);
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    liftMotor.setTargetPosition(0);
                })
                .back(5)
                .build();

        TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(scoreTraj.end())
                .setVelConstraint(angCon)
                .lineToLinearHeading(new Pose2d(startTraj.end().getX(), startTraj.end().getY() - 1.75, Math.toRadians(-90)))
                .forward(22.3)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    clawServo.setPosition(LiftClaw.ClawModes.CLOSED.position);
                })
                .waitSeconds(0.5)
                .lineToLinearHeading(startTraj.end())
                .resetVelConstraint()
                .build();

        AprilTagDetection detectedTag = null;
        while (!isStopRequested() && !opModeIsActive()) {
            if (detectedTag == null)
                detectedTag = detectionSystem.detect();
            else
                telemetry.addData("Detected Tag ID: " + detectedTag.id, " ");

            telemetry.update();
        }



        detectionSystem.closeCamera();

        drive.followTrajectorySequence(startTraj);

        drive.followTrajectorySequenceAsync(scoreTraj);
        while (drive.isBusy()) {
            liftMotor.updatePosition();
            drive.update();
        }

        drive.followTrajectorySequenceAsync(cycleTrajectory);
        while (drive.isBusy()) {
            liftMotor.updatePosition();
            drive.update();
        }

        drive.followTrajectorySequenceAsync(scoreTraj);
        while (drive.isBusy()) {
            liftMotor.updatePosition();
            drive.update();
        }

        drive.followTrajectorySequenceAsync(cycleTrajectory);
        while (drive.isBusy()) {
            liftMotor.updatePosition();
            drive.update();
        }

        drive.followTrajectorySequenceAsync(scoreTraj);
        while (drive.isBusy()) {
            liftMotor.updatePosition();
            drive.update();
        }

        sleep(1000);


        return;
    }
}
