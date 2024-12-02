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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-5, 65, Math.toRadians(90)))

                        .setReversed(true)
                        .splineTo(new Vector2d(8, 31.5), Math.toRadians(270))  //place specimen

        // once at -5, 36, open claw

        //pivotservo set to specimen
        //slides set to specimen
        //action

                .setReversed(false)
                .splineTo(new Vector2d(-35,37 ), Math.toRadians(270)) //waypoint to first sample

                .splineTo(new Vector2d(-50, 6), Math.toRadians(270)) // align with first sampleample


                .setReversed(true)
                .splineTo(new Vector2d(-46,53), Math.toRadians(90))// push first sample
        //.setReversed(false) ;// push first sample

        //        .splineTo(new Vector2d(-47, 4), Math.toRadians(270))  // return
        //         .setReversed(true)
        //         .splineTo(new Vector2d(-58, 22), Math.toRadians(90)) // waypoint
        //         .splineTo(new Vector2d( -58, 53), Math.toRadians(90)); //push 2nd  //push 2nd


        //cycle 2nd specimen

                .waitSeconds(0.1)
                .strafeTo(new Vector2d(-46, 61))// position for intaking 2nd
                .waitSeconds(.3)




                .setTangent(Math.toRadians(-90))
                        .splineTo(new Vector2d(-37, 50), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-5, 50, Math.toRadians(90)), Math.toRadians(270))
                .waitSeconds(.1)
                .strafeTo(new Vector2d(-5, 30.7))
                .waitSeconds(.2) // place 2nd specimen

                .setTangent(90)
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking 2nd

                .waitSeconds(.4)
                .strafeTo(new Vector2d(-37, 61))
                .waitSeconds(.4)              //grab 3rd


                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)), Math.toRadians(270)) //intak
                .waitSeconds(.1)
                .strafeTo(new Vector2d(5, 30.7))
                .setTangent(90)
                .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                //grab 2nd
                .waitSeconds(.4)
                .strafeTo(new Vector2d(-37, 61))
                .waitSeconds(.4)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, 50, Math.toRadians(90)), Math.toRadians(270)) //intak
                .waitSeconds(.1)
                .strafeTo(new Vector2d(-12, 30.7))
                .waitSeconds(.2)
                .strafeTo(new Vector2d(-12, 30.5))
                .build());// place 4th specimen // 4th cycle


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}