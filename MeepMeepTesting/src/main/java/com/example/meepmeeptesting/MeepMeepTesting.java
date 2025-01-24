package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double test = -35.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)

                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-20, -60, Math.toRadians(270)))
                        .waitSeconds(3)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(5, -31), Math.toRadians(90))
                .waitSeconds(.3)
                .setTangent(Math.toRadians(270))
                .splineTo(new Vector2d(35,-37 ), Math.toRadians(90)) //waypoint to first sample
                .splineToConstantHeading(new Vector2d(35, -20), Math.toRadians(90)) // align with first sampleample

                .splineToConstantHeading(new Vector2d(46, -7), Math.toRadians(90)) // align with first sampleample
                .splineToConstantHeading(new Vector2d(46, -40), Math.toRadians(270)) //push first
                .splineToConstantHeading(new Vector2d(46, -50), Math.toRadians(270)) //push first

                .splineToConstantHeading(new Vector2d(50, -15), Math.toRadians(270)) // return
                .splineToConstantHeading(new Vector2d(62, -20), Math.toRadians(270)) // waypoint to push second // waypoint to push second
                .splineToConstantHeading(new Vector2d(62, -40.5), Math.toRadians(270)) // push second and grab
                .splineToConstantHeading(new Vector2d(62, -60.5), Math.toRadians(270)) // push second and grab


                .waitSeconds(.1) //grab spec
                .waitSeconds(.1)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(0, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineTo(new Vector2d(0, -31), Math.toRadians(90))

                .waitSeconds(.3)
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(47, -52, Math.toRadians(90)), Math.toRadians(270))
                .splineTo(new Vector2d(47, -60.75), Math.toRadians(270))

                .waitSeconds(.1)
                .waitSeconds(.1)

                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-3, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineTo(new Vector2d(-3, -31), Math.toRadians(90))

                .waitSeconds(.3)

                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(47, -52, Math.toRadians(90)), Math.toRadians(270))
                .splineTo(new Vector2d(47, -60.75), Math.toRadians(270))


                .waitSeconds(.1)
                .waitSeconds(.1)

                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(3, -45, Math.toRadians(-90)), Math.toRadians(90))
                .splineTo(new Vector2d(3, -31), Math.toRadians(90))
                .waitSeconds(.3 )
                .strafeTo(new Vector2d(2, -40))

                        /*
                               .waitSeconds(15)
                               .strafeTo(new Vector2d(-37,-50))
                               .setReversed(true)
                               .setTangent(Math.toRadians(225))
                               .splineTo(new Vector2d(-56.5, -56.5), Math.toRadians(180))
                               .waitSeconds(.5)
                               .setReversed(false)
                               .setTangent(Math.toRadians(90))
                               .splineTo(new Vector2d(-49, -50), Math.toRadians(270))

                               .waitSeconds(.7)
                               .strafeTo(new Vector2d(-49, test)) //TODO
                               .waitSeconds(.2)
                               .waitSeconds(3)
                               .setReversed(true)
                               .waitSeconds(.5)
                               .lineToLinearHeading(new Pose2d(-56.5, -56.5))
                               .waitSeconds(.5)
                               .lineToLinearHeading(new Pose2d(-60, -50))
                               .waitSeconds(.7)
                               .lineToLinearHeading(new Pose2d(-60, test)) //TODO
                               .waitSeconds(3)

                               .waitSeconds(.5)
                               .lineToLinearHeading(new Pose2d(-56.5, -56.5))
                               .waitSeconds(.5)
                               .lineToLinearHeading(new Pose2d(-50, -50))
                               .waitSeconds(.2)
                               .lineToLinearHeading(new Pose2d(-59, -27.3))
                               .waitSeconds(.5)
                               .lineToLinearHeading(new Pose2d(-44, -27.3))
                               .waitSeconds(1.3)
                               .lineToLinearHeading(new Pose2d(-53, -53))
                               .strafeTo(new Vector2d(-56.5,-56.5 ))
                               .waitSeconds(.5)
                               .lineToLinearHeading(new Pose2d(-50, -50))
                               // actions that need to happen on init; for insta
                        */
                         .build());// place 4th specimen // 4th cycle


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}