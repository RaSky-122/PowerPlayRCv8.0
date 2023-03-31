package org.firstinspires.ftc.teamcode.rasky.autonomy;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rasky.components.LiftClaw;
import org.firstinspires.ftc.teamcode.rasky.components.LiftNewClaw;
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
@Autonomous(name = "Red Right", group = "tests")
public class RedRight extends LinearOpMode {

    AutonomyDetection detectionSystem;
    VoltageSensor voltageSensor;
    WrappedServo servoClaw;
    WrappedServo servoJoint;
    //adaugat de raca , nu te speria
    WrappedServo servoAxle;
    ElapsedTime timer = new ElapsedTime();

    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL / 1.5,
            DriveConstants.MAX_ANG_VEL / 3,
            DriveConstants.TRACK_WIDTH);

    TrajectoryVelocityConstraint velCon2 = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL / 1.5,
            DriveConstants.MAX_ANG_VEL / 1.5,
            DriveConstants.TRACK_WIDTH);

    TrajectoryVelocityConstraint velNorm = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.TRACK_WIDTH);

    DcMotorEx liftMotor;
    DcMotorEx liftMotor2;

    PIDFController controller;
    double p = 0.007, i = 0.003, d = 0.001;
    double f = 0.15;

    int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(40.5, -58, Math.toRadians(-90)));

        detectionSystem = new AutonomyDetection(hardwareMap);
        detectionSystem.init();

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(new PIDCoefficients(p, i, d));

        servoClaw = new WrappedServo(hardwareMap);
        servoClaw.Init("claw", false, true);
        servoClaw.setPWMRange(500, 2500);
        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);

        servoJoint = new WrappedServo(hardwareMap);
        servoJoint.Init("joint", false, false);
        servoJoint.setPWMRange(500, 2500);
        servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);

        servoAxle = new WrappedServo(hardwareMap);
        servoAxle.Init("axle", false, true);
        servoAxle.setPWMRange(500, 2500);

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setVelConstraint(velCon2)
                .setReversed(true)
                .addTemporalMarker(1.2, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 20;
                })
                .addTemporalMarker(1.5, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .splineToSplineHeading(new Pose2d(35.5, -30, Math.toRadians(-70)), Math.toRadians(90))
                .setVelConstraint(velCon)
                .splineToSplineHeading(new Pose2d(26.5, -6.5, Math.toRadians(-50)), Math.toRadians(145))
                .build();

        TrajectorySequence twoTraj = drive.trajectorySequenceBuilder(startTraj.end())
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    target = 310;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .setVelConstraint(velCon)
                .splineToLinearHeading(new Pose2d(59.5, -11.5, Math.toRadians(0)), Math.toRadians(-30))
                .build();

        TrajectorySequence threeTraj = drive.trajectorySequenceBuilder(twoTraj.end())
                .setVelConstraint(velCon2)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 900;
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.35, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 20;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(29, -4.5, Math.toRadians(-28)))
                .build();

        TrajectorySequence fourTraj = drive.trajectorySequenceBuilder(threeTraj.end())
                .setVelConstraint(velCon2)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    target = 290;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence fiveTraj = drive.trajectorySequenceBuilder(fourTraj.end())
                .setVelConstraint(velCon2)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 900;
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.35, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 20;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(30, -4.5, Math.toRadians(-28)))
                .build();

        TrajectorySequence sixTraj = drive.trajectorySequenceBuilder(fiveTraj.end())
                .setVelConstraint(velCon2)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    target = 200;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence sevenTraj = drive.trajectorySequenceBuilder(sixTraj.end())
                .setVelConstraint(velCon2)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 900;
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.35, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 20;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(29.5, -6, Math.toRadians(-30)))
                .build();

        TrajectorySequence eightTraj = drive.trajectorySequenceBuilder(sevenTraj.end())
                .setVelConstraint(velCon2)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    target = 120;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence nineTraj = drive.trajectorySequenceBuilder(eightTraj.end())
                .setVelConstraint(velCon2)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 900;
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.35, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 20;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(29, -6, Math.toRadians(-30)))
                .build();

        TrajectorySequence tenTraj = drive.trajectorySequenceBuilder(nineTraj.end())
                .setVelConstraint(velCon2)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.1, () -> {
                    target = 40;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(60, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .build();

        TrajectorySequence elevenTraj = drive.trajectorySequenceBuilder(tenTraj.end())
                .setVelConstraint(velCon2)
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 850;
                })
                .addTemporalMarker(0.1, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
                })
                .addTemporalMarker(0.4, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position +10;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(29, -6.5, Math.toRadians(-28)))
                .build();


        TrajectorySequence parkZoneOne = drive.trajectorySequenceBuilder(elevenTraj.end())
                .forward(11)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                    target = 0;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .lineToLinearHeading(new Pose2d(12, -15, Math.toRadians(90)))
                .build();

        TrajectorySequence parkZoneTwo = drive.trajectorySequenceBuilder(elevenTraj.end())
                .forward(11)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                    target = 0;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .lineToLinearHeading(new Pose2d(35, -15, Math.toRadians(90)))
                .build();

        TrajectorySequence parkZoneThree = drive.trajectorySequenceBuilder(elevenTraj.end())
                .forward(11)
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                    target = 0;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .lineToLinearHeading(new Pose2d(57, -15, Math.toRadians(90)))
                .build();

        AprilTagDetection detectedTag = null;
        while (!isStopRequested() && !opModeIsActive()) {
            detectedTag = detectionSystem.detect();
            if (detectedTag != null)
                telemetry.addData("Detected Tag ID: " + detectedTag.id, " ");

            telemetry.update();
        }
        ElapsedTime timer = new ElapsedTime();
        detectionSystem.closeCamera();

        drive.followTrajectorySequenceAsync(startTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(200);

        drive.followTrajectorySequenceAsync(twoTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        timer.reset();
        while (timer.milliseconds() <= 400)
            liftPID();

        drive.followTrajectorySequenceAsync(threeTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(170);

        drive.followTrajectorySequenceAsync(fourTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        timer.reset();
        while (timer.milliseconds() <= 400)
            liftPID();

        drive.followTrajectorySequenceAsync(fiveTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(170);

        drive.followTrajectorySequenceAsync(sixTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        timer.reset();
        while (timer.milliseconds() <= 400)
            liftPID();

        drive.followTrajectorySequenceAsync(sevenTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(170);

        drive.followTrajectorySequenceAsync(eightTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        timer.reset();
        while (timer.milliseconds() <= 400)
            liftPID();

        drive.followTrajectorySequenceAsync(nineTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(170);

//        drive.followTrajectorySequenceAsync(tenTraj);
//        while (drive.isBusy()) {
//            liftPID();
//            liftPID();
//            drive.update();
//            liftPID();
//            liftPID();
//        }
//
//        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
//        timer.reset();
//        while (timer.milliseconds() <= 400)
//            liftPID();
//
//        drive.followTrajectorySequenceAsync(elevenTraj);
//        while (drive.isBusy()) {
//            liftPID();
//            liftPID();
//            drive.update();
//            liftPID();
//            liftPID();
//        }
//
//        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
//        sleep(170);

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
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        return;
    }

    void liftPID() {
        controller.setTargetPosition(target);
        int currPos = liftMotor.getCurrentPosition();
        double pidVal = controller.update(currPos);

        double ff = 1 * f;
        double power = ff + pidVal;

        liftMotor.setPower(power * 0.85);
        liftMotor2.setPower(power * 0.85);
    }
}
