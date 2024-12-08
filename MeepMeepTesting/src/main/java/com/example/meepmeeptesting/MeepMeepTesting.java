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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(5, -65, Math.toRadians(90)))
                        .setReversed(true)
                        .splineTo(new Vector2d(5, -30.4), Math.toRadians(90))  //waypoint to first sample

        // once at -5, 36, open claw

        //pivotservo set to specimen
        //slides set to specimen
        //action
                        .setReversed(false)
                     .splineTo(new Vector2d(35,-37 ), Math.toRadians(90)) //waypoint to first sample

                         .splineTo(new Vector2d(50, -1), Math.toRadians(90)) // align with first sampleample


                         .setReversed(true)
                         .splineTo(new Vector2d(45,-53), Math.toRadians(270))// push first sample

        // actions that need to happen on init; for insta
                         .build());// place 4th specimen // 4th cycle


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}