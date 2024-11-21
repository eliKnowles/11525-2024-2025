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
                .setConstraints(60, 60, 60, 60, 15.7)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-26, 63, 4.71239))
                        .splineTo(new Vector2d(-36, 13), 4.71239)
                        //go to starting pos for sample pusher
                        .strafeTo(new Vector2d(-47,13))
                        //align to sample
                        .strafeTo(new Vector2d(-47,52))
                        // push sample
                        .strafeTo(new Vector2d(-47,13))
                        //return to position
                        .strafeTo(new Vector2d(-57, 13))
                        //align to sample
                        .strafeTo(new Vector2d(-57, 52))
                        //push sample
                        .strafeTo(new Vector2d(-57, 13))
                        //return to position
                        .strafeTo(new Vector2d(-61, 13))
                        //align to sample
                        .strafeTo(new Vector2d(-61, 52))
                        //push sample
                        .strafeTo(new Vector2d(-42,54))

                        .build());



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}