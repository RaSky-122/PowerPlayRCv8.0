package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveShim;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setStartPose(new Pose2d(41, -58, Math.toRadians(-90)))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(11.08, 14.05)
                .build();

        DriveShim drive = myBot.getDrive();


        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(35, -30, Math.toRadians(-70)), Math.toRadians(90))
                //.setVelConstraint(velCon)
                .splineToSplineHeading(new Pose2d(28, -5.5, Math.toRadians(-48)), Math.toRadians(150))
                .build();

        TrajectorySequence twoTraj = drive.trajectorySequenceBuilder(startTraj.end())
                .splineToLinearHeading(new Pose2d(59.5, -11.5, Math.toRadians(0)), Math.toRadians(-40))
                .build();

        TrajectorySequence threeTraj = drive.trajectorySequenceBuilder(twoTraj.end())
                .lineToLinearHeading(new Pose2d(-28.25, -16.75, Math.toRadians(130)))
                .build();

        TrajectorySequence fourTraj = drive.trajectorySequenceBuilder(threeTraj.end())
                .splineToLinearHeading(new Pose2d(-60.25, -10.5, Math.toRadians(-90)), Math.toRadians(-90))
                .build();


        myBot.followTrajectorySequence(startTraj);
        myBot.followTrajectorySequence(twoTraj);
        //myBot.followTrajectorySequence(threeTraj);
        //myBot.followTrajectorySequence(fourTraj);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}