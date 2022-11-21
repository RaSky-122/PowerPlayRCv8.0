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
                .setStartPose(new Pose2d(35.5, -62, Math.toRadians(90)))
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(11.08, 14.05)
                .build();

        DriveShim drive = myBot.getDrive();

        TrajectorySequence startTraj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(55)
                .back(3)
                .turn(Math.toRadians(45))
                .build();

        TrajectorySequence cycleTrajectory = drive.trajectorySequenceBuilder(startTraj.end())
                .forward(9)
                .lineToLinearHeading(new Pose2d(startTraj.end().getX(), startTraj.end().getY() - 1.75, Math.toRadians(0)))
                .forward(25)
                .build();

        myBot.followTrajectorySequence(startTraj);
        myBot.followTrajectorySequence(cycleTrajectory);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}