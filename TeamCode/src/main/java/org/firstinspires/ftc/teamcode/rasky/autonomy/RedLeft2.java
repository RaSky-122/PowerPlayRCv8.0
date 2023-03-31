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
@Autonomous(name = "Red Left 2", group = "tests")
public class RedLeft2 extends LinearOpMode {

    AutonomyDetection detectionSystem;
    VoltageSensor voltageSensor;
    WrappedServo servoClaw;
    WrappedServo servoJoint;
    //adaugat de raca , nu te speria
    WrappedServo servoAxle;
    ElapsedTime timer = new ElapsedTime();

    TrajectoryVelocityConstraint velCon = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.TRACK_WIDTH);

    TrajectoryVelocityConstraint angCon = SampleMecanumDrive.getVelocityConstraint(
            DriveConstants.MAX_VEL,
            DriveConstants.MAX_ANG_VEL / 3,
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
        drive.setPoseEstimate(new Pose2d(-41, -58, Math.toRadians(-90)));

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
                .setReversed(true)
                .addTemporalMarker(1.1, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position - 30;
                })
                .addTemporalMarker(1.5, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .splineToSplineHeading(new Pose2d(-26, -37, Math.toRadians(-145)), Math.toRadians(30))
                .splineToSplineHeading(new Pose2d(-6, -22, Math.toRadians(-180)), Math.toRadians(20))
                .build();

        TrajectorySequence twoTraj = drive.trajectorySequenceBuilder(startTraj.end())
                .addTemporalMarker(0.5, () -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(1, () -> {
                    target = 370;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(-18, -8, Math.toRadians(-180)), Math.toRadians(-200))
                .splineToLinearHeading(new Pose2d(-59, -7.5, Math.toRadians(-180)), Math.toRadians(-180))
                .build();

        TrajectorySequence threeTraj = drive.trajectorySequenceBuilder(twoTraj.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 700;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                })
                .addTemporalMarker(1.4, () -> {
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position - 30;
                })
                .lineToSplineHeading(new Pose2d(-25, -9, Math.toRadians(-180)))
                .splineToSplineHeading(new Pose2d(-7, -19, Math.toRadians(-220)), Math.toRadians(-50))
                .build();

        TrajectorySequence fourTraj = drive.trajectorySequenceBuilder(threeTraj.end())
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.5, () -> {
                    target = 330;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(-59, -6, Math.toRadians(-180)), Math.toRadians(-190))
                .build();

        TrajectorySequence fiveTraj = drive.trajectorySequenceBuilder(fourTraj.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 700;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                })
                .addTemporalMarker(1.4, () -> {
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position - 30;
                })
                .lineToSplineHeading(new Pose2d(-25, -9, Math.toRadians(-180)))
                .splineToSplineHeading(new Pose2d(-7, -18.5, Math.toRadians(-215)), Math.toRadians(-50))
                .build();

        TrajectorySequence sixTraj = drive.trajectorySequenceBuilder(fiveTraj.end())
                .addTemporalMarker(() -> {
                    servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
                })
                .addTemporalMarker(0.5, () -> {
                    target = 240;
                    servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
                })
                .splineToLinearHeading(new Pose2d(-59, -5.75, Math.toRadians(-180)), Math.toRadians(-190))
                .build();

        TrajectorySequence sevenTraj = drive.trajectorySequenceBuilder(sixTraj.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    target = 700;
                    servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);
                })
                .addTemporalMarker(1.4, () -> {
                    target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position - 30;
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToSplineHeading(new Pose2d(-25, -9, Math.toRadians(-180)))
                .splineToSplineHeading(new Pose2d(-6, -14, Math.toRadians(-225)), Math.toRadians(-50))
                .build();

        AprilTagDetection detectedTag = null;
        while (!isStopRequested() && !opModeIsActive()) {
            //detectedTag = detectionSystem.detect();
            if (detectedTag != null)
                telemetry.addData("Detected Tag ID: " + detectedTag.id, " ");

            telemetry.update();
        }
        ElapsedTime timer = new ElapsedTime();
        //detectionSystem.closeCamera();

        drive.followTrajectorySequenceAsync(startTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        sleep(170);
        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(170);

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

        return;
    }

    void liftPID() {
        controller.setTargetPosition(target);
        int currPos = liftMotor.getCurrentPosition();
        double pidVal = controller.update(currPos);

        double ff = 1 * f;
        double power = ff + pidVal;

        liftMotor.setPower(power * 0.8);
        liftMotor2.setPower(power * 0.8);
    }
}
