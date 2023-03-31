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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
@Autonomous(name = "Test Autonomy2", group = "tests")
public class TestAutonomy2 extends LinearOpMode {

    AutonomyDetection detectionSystem;
    WrappedServo servoClaw;
    WrappedServo servoJoint;
    //adaugat de raca , nu te speria
    WrappedServo servoAxle;
    ElapsedTime timer = new ElapsedTime();

    DcMotorEx liftMotor;
    DcMotorEx liftMotor2;

    PIDFController controller;
    double p = 0.009, i = 0, d = 0.001;
    double f = 0.15;

    int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(30, -62, Math.toRadians(90)));

        detectionSystem = new AutonomyDetection(hardwareMap);
        detectionSystem.init();

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
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
                .lineToLinearHeading(new Pose2d(32, -54.5, Math.toRadians(125)))
                .build();

        TrajectorySequence twoTraj = drive.trajectorySequenceBuilder(startTraj.end())
                .strafeRight(5)
                .lineToLinearHeading(new Pose2d(35, -8.75, Math.toRadians(0)))
                .build();

        TrajectorySequence threeTraj = drive.trajectorySequenceBuilder(twoTraj.end())
                .lineToConstantHeading(new Vector2d(60, -9))
                .build();

        TrajectorySequence fourTraj = drive.trajectorySequenceBuilder(threeTraj.end())
                .addTemporalMarker(0.3, () -> {
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(28.5, -8, Math.toRadians(-60)))
                .build();

        TrajectorySequence fiveTraj = drive.trajectorySequenceBuilder(fourTraj.end())
                .lineToLinearHeading(new Pose2d(59.8, -10, Math.toRadians(0)))
                .build();

        TrajectorySequence sixTraj = drive.trajectorySequenceBuilder(fiveTraj.end())
                .addTemporalMarker(0.3, () -> {
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(28.7, -8.5, Math.toRadians(-60)))
                .build();

        TrajectorySequence sevenTraj = drive.trajectorySequenceBuilder(sixTraj.end())
                .lineToLinearHeading(new Pose2d(59.8, -10, Math.toRadians(0)))
                .build();

        TrajectorySequence eightTraj = drive.trajectorySequenceBuilder(fiveTraj.end())
                .addTemporalMarker(0.3, () -> {
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(28.7, -9.5, Math.toRadians(-60)))
                .build();

        TrajectorySequence nineTraj = drive.trajectorySequenceBuilder(sixTraj.end())
                .lineToLinearHeading(new Pose2d(59.8, -10, Math.toRadians(0)))
                .build();

        TrajectorySequence tenTraj = drive.trajectorySequenceBuilder(fiveTraj.end())
                .addTemporalMarker(0.3, () -> {
                    servoJoint.setPosition(LiftNewClaw.JointModes.BACK.position);
                })
                .lineToLinearHeading(new Pose2d(28.7, -10, Math.toRadians(-60)))
                .build();

        TrajectorySequence parkZoneOne = drive.trajectorySequenceBuilder(tenTraj.end())
                .lineToLinearHeading(new Pose2d(12, -13, Math.toRadians(90)))
                .build();

        TrajectorySequence parkZoneTwo = drive.trajectorySequenceBuilder(tenTraj.end())
                .lineToLinearHeading(new Pose2d(35, -13, Math.toRadians(90)))
                .build();

        TrajectorySequence parkZoneThree = drive.trajectorySequenceBuilder(tenTraj.end())
                .lineToLinearHeading(new Pose2d(57, -13, Math.toRadians(90)))
                .build();

        AprilTagDetection detectedTag = null;
        while (!isStopRequested() && !opModeIsActive()) {
            detectedTag = detectionSystem.detect();
            if (detectedTag != null)
                telemetry.addData("Detected Tag ID: " + detectedTag.id, " ");

            telemetry.update();
        }

        detectionSystem.closeCamera();

        target = 30;
        drive.followTrajectorySequenceAsync(startTraj);
        servoAxle.setPosition(LiftSystem.LiftPositions.LOW_JUNCTION.axlePos);
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
            drive.update();
        }

        drive.followTrajectorySequenceAsync(threeTraj);
        servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
        target = 360;

        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        sleep(250);
        target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 50;
        liftPID();

        servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);

        drive.followTrajectorySequenceAsync(fourTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        drive.followTrajectorySequenceAsync(fiveTraj);
        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(250);
        servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
        servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
        sleep(100);
        target = 260;

        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        sleep(250);
        target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 60;
        liftPID();

        servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);

        drive.followTrajectorySequenceAsync(sixTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(250);
        servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
        servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
        sleep(100);
        target = 170;

        drive.followTrajectorySequenceAsync(sevenTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        sleep(250);
        target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 70;
        liftPID();

        servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);

        drive.followTrajectorySequenceAsync(eightTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(250);
        servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
        servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
        sleep(100);
        target = 95;

        drive.followTrajectorySequenceAsync(nineTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.CLOSED.position);
        sleep(250);
        target = (int) LiftSystem.LiftPositions.HIGH_JUNCTION.position + 80;
        liftPID();

        servoAxle.setPosition(LiftSystem.LiftPositions.HIGH_JUNCTION.axlePos);

        drive.followTrajectorySequenceAsync(tenTraj);
        while (drive.isBusy()) {
            liftPID();
            liftPID();
            drive.update();
            liftPID();
            liftPID();
        }

        servoClaw.setPosition(LiftNewClaw.ClawModes.OPEN.position);
        sleep(250);
        servoAxle.setPosition(LiftSystem.LiftPositions.STARTING_POS.axlePos);
        servoJoint.setPosition(LiftNewClaw.JointModes.FRONT.position);
        sleep(100);
        target = 0;

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

        liftMotor.setPower(power);
        liftMotor2.setPower(power);
    }
}
