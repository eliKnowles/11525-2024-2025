package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(37, 65, Math.toRadians(270)))
                                .strafeTo(new Vector2d(37,50))
                                .setReversed(true)
                                .setTangent(Math.toRadians(45))
                                .splineTo(new Vector2d(55.5, 55.5), Math.toRadians(0))
                                .waitSeconds(.8)
                                .strafeTo(new Vector2d(55.7,55.7))
                                .waitSeconds(.4)
                                .setReversed(false)
                                .splineTo(new Vector2d(49,50),Math.toRadians(270))
                                .strafeTo(new Vector2d(49,39.3 ))
                                .waitSeconds(1)
                                .setReversed(true)
                                .setTangent(Math.toRadians(45))
                                 .splineToLinearHeading(new Pose2d(55.5, 55.5, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .waitSeconds(.8)
                                 .setTangent(Math.toRadians(-90))
                                .splineTo(new Vector2d(58,47),Math.toRadians(-45))
                                .strafeTo(new Vector2d(58,39.3 ))
                                .waitSeconds(1)
                                .setReversed(true)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(new Pose2d(55.5, 55.5, Math.toRadians(225)), Math.toRadians(45))
                                .setReversed(false)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(53, 27, Math.toRadians(0)), Math.toRadians(0))
                                .strafeTo(new Vector2d (57, 27))


                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(55.5, 55.5, Math.toRadians(225)), Math.toRadians(45))





        // actions that need to happen on init; for insta
                         .build());// place 4th specimen // 4th cycle


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}