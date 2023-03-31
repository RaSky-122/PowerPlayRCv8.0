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
@Autonomous(name = "Blue Right", group = "tests")
public class BlueRight extends LinearOpMode {

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
        liftMotor.Init("liftMotor", true, false, true, true, true);

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

        TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL / 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);
        TrajectoryVelocityConstraint angCon = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL * 1.5, DriveConstants.TRACK_WIDTH);

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(55.6)
                .addTemporalMarker(() -> {
                    liftMotor.setTargetPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.position);
                })
                .waitSeconds(0.4)
                .strafeLeft(13.2)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    clawServo.setPosition(LiftClaw.ClawModes.OPEN.position);
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence collectTraj = drive.trajectorySequenceBuilder(startTraj.end())
                .back(3)
                .addTemporalMarker(() -> {
                    stackAdjust();
                })
                .turn(Math.toRadians(-90))
                .forward(36.7)
                .addTemporalMarker(() -> {
                    clawServo.setPosition(LiftClaw.ClawModes.CLOSED.position);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    liftMotor.setTargetPosition(LiftSystem.LiftPositions.LOW_JUNCTION.position);
                })
                .waitSeconds(0.25)
                .build();

        TrajectorySequence travelTraj = drive.trajectorySequenceBuilder(collectTraj.end())
                .back(36.7)
                .addTemporalMarker(() -> {
                    liftMotor.setTargetPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.position);
                })
                .turn(Math.toRadians(90))
                .forward(1.5)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    clawServo.setPosition(LiftClaw.ClawModes.OPEN.position);
                })
                .waitSeconds(0.1)
                .build();


        TrajectorySequence prePark = drive.trajectorySequenceBuilder(startTraj.end())
                .back(5)
                .addTemporalMarker(() -> {
                    liftMotor.setTargetPosition(LiftSystem.LiftPositions.STARTING_POS.position);
                })
                .build();

        TrajectorySequence parkZoneOne = drive.trajectorySequenceBuilder(prePark.end())
                .strafeLeft(10)
                .build();

        TrajectorySequence parkZoneTwo = drive.trajectorySequenceBuilder(prePark.end())
                .strafeRight(10)
                .build();

        TrajectorySequence parkZoneThree = drive.trajectorySequenceBuilder(prePark.end())
                .strafeRight(35)
                .build();

        AprilTagDetection detectedTag = null;
        while (!isStopRequested() && !opModeIsActive()) {
            detectedTag = detectionSystem.detect();
            if (detectedTag != null)
                telemetry.addData("Detected Tag ID: " + detectedTag.id, " ");

            telemetry.update();
        }

        detectionSystem.closeCamera();

        drive.followTrajectorySequenceAsync(startTraj);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        drive.followTrajectorySequenceAsync(collectTraj);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        drive.followTrajectorySequenceAsync(travelTraj);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        drive.followTrajectorySequenceAsync(collectTraj);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        drive.followTrajectorySequenceAsync(travelTraj);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        drive.followTrajectorySequenceAsync(prePark);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        if (detectedTag != null) {
            if (detectedTag.id == 0)
                drive.followTrajectorySequenceAsync(parkZoneOne);
            else if (detectedTag.id == 1)
                drive.followTrajectorySequenceAsync(parkZoneTwo);
            else
                drive.followTrajectorySequenceAsync(parkZoneThree);
        } else
            drive.followTrajectorySequenceAsync(parkZoneThree);

        while (drive.isBusy()) {
            drive.update();
            liftMotor.updatePosition();
        }

        return;
    }

    double stackHeight = 235;

    void stackAdjust() {
        liftMotor.setTargetPosition(stackHeight);
        stackHeight -= 60;
    }
}
