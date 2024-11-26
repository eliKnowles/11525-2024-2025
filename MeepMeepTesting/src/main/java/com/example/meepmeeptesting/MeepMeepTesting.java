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
                .setConstraints(85, 57, 70, 70, 14.5)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-5, 63, Math.toRadians(90)))

                        .setReversed(true)
                        .splineTo(new Vector2d(-5, 32), Math.toRadians(270)) //waypoint to first sample

        // once at -5, 36, open claw

        //pivotservo set to specimen
        //slides set to specimen
        //action
                        
                        .setReversed(false)
                        .splineTo(new Vector2d(-33, 37), Math.toRadians(270)) //waypoint to first sample

                         .splineTo(new Vector2d(-45, 6), Math.toRadians(270)) // align with first sampleample


                        .setReversed(true)
                         .splineTo(new Vector2d(-46,53), Math.toRadians(90))// push first sample
                        .setReversed(false)// push first sample

                         .splineTo(new Vector2d(-47, 4), Math.toRadians(270))  // return
                        .setReversed(true)
                        .splineTo(new Vector2d(-58, 22), Math.toRadians(90)) // waypoint
                        .splineTo(new Vector2d( -58, 53), Math.toRadians(90)) //push 2nd


        //cycle 2nd specimen


                      //  .setTangent(Math.toRadians(90))
                       // .splineTo(new Vector2d(-37, 58), Math.toRadians(90)) //intake
                        .strafeTo(new Vector2d(-37,58 ))
                        .waitSeconds(.5)

                        .strafeTo(new Vector2d(-37, 63))
                        .waitSeconds(.5)





                         .setReversed(false)
                        .setTangent(Math.toRadians(-90))
                         .splineToLinearHeading(new Pose2d(-2, 32, Math.toRadians(90)), Math.toRadians(270)) //intak







        //cycle 2nd specimen
                      /*  .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                        //grab 2nd
                        .waitSeconds(.2)
                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-37, 57), Math.toRadians(90))


                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-37, 63), Math.toRadians(90)) //intake

                        // cycle 3rd specimen
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                        .setReversed(false)

                        //grab 3rd
                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-37, 57), Math.toRadians(90))
                        .waitSeconds(.2) // grab

                        // place
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
                        .waitSeconds(.2)

                        // cycle 4th specimen
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                        .setReversed(false)

                        //// grab
                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-37, 57), Math.toRadians(90))
                        .waitSeconds(.2)

                        // place
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
                        .waitSeconds(.2)



                        //  BELOW == 5 SPECIMEN
                        .setReversed(true)
                        .setTangent(Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking

                        .setTangent(Math.toRadians(90))
                        .splineTo(new Vector2d(-37, 57), Math.toRadians(90))
                        .waitSeconds(.1) // grab

                        .setTangent(Math.toRadians(0))
                        .splineToLinearHead*/
//







                        .build());


//                        .setReversed(true)
//                        .splineTo(new Vector2d(-5, 36), Math.toRadians(270))
//                        .waitSeconds(.2)
//                        //action
//
//                        .setReversed(false)
//                        .splineTo(new Vector2d(-35, 28), Math.toRadians(270))
//
//                        .splineTo(new Vector2d(-45, 6), Math.toRadians(270)) // align with first sample
//
//                        .strafeTo(new Vector2d(-45,53)) // push first sample
//
//                        .strafeTo(new Vector2d(-47, 11)) // drive back for second
//                        .setReversed(true)
//                        .splineTo(new Vector2d(-55, 25), Math.toRadians(90)) //waypoint
//
//                        .splineTo(new Vector2d( -55, 53), Math.toRadians(90)) //pushed
//                        .setReversed(false)
////                        .splineTo(new Vector2d( -57, 11), Math.toRadians(270)) //return for last
//                        .setReversed(true)
////                        .strafeTo(new Vector2d(-61,11))  //align with last sample
////                        .strafeTo(new Vector2d(-59,53)) // push last sample               // DISABLE THIS PATH IF YOU WANT A RELIABLE 4 SPECIMEN
//
//
//
//                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//
//                        .setReversed(false)
//                        .waitSeconds(.3)
//                        .strafeTo(new Vector2d(-37, 57)) //intake
//
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                        .waitSeconds(.2)
//
//                        .setReversed(true)
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//                        .setReversed(false)
//                        .strafeTo(new Vector2d(-37, 57))
//                        .waitSeconds(.2) // grab
//
//
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                        .waitSeconds(.2)
//
//
//                        .setReversed(true)
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//                        .setReversed(false)
//                        .strafeTo(new Vector2d(-37, 57))
//                        .waitSeconds(.3) // grab
//
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                        .waitSeconds(.3)
//
//                        .setReversed(true)
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//                        .setReversed(false)
//                        .strafeTo(new Vector2d(-37, 57))
//                        .waitSeconds(.1) // grab
//
//                        .setTangent(Math.toRadians(0))
//                        .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
////                        .waitSeconds(.1)
//
//
//
//
//
////                        .splineTo(new Vector2d( -37, 55 ), Math.toRadians(-90)) // drive to wing, open claw, prepare for specimen
////                        .waitSeconds(.8) // wait for human player to align specimen
////                        .strafeTo(new Vector2d(-37, 59)) //drive to specimen, close claw, and go to score
////                        // TODO, close claw action, go to scoring position action
////                        .waitSeconds(.5)
////                        .splineTo(new Vector2d(-5, 36), Math.toRadians(-270)) // place
////                        .waitSeconds(.5)//return for last
////                        .splineTo(new Vector2d( -37, 55 ), Math.toRadians(-90)) // preapre to grab second
////                        .waitSeconds(.5)
////                        .strafeTo(new Vector2d(-37, 59)) //grab second
////                        .waitSeconds(.2)
////                        .splineTo(new Vector2d(-5, 36), Math.toRadians(-270))
////                        .waitSeconds(.5)
////                        .splineTo(new Vector2d( -37, 55 ), Math.toRadians(-90))
////                        .waitSeconds(.5)
////                        .strafeTo(new Vector2d(-37, 59)) //grab second
////                        .waitSeconds(.2)
////                        .splineTo(new Vector2d(-5, 36), Math.toRadians(-270))
////                        .waitSeconds(.5)
////                        .setReversed(false)
//
//                                .build());
//                        // .splineTo(new Vector2d(-48, 13), Math.toRadians(0))
//
//
//
//                        //go to starting pos for sample pusher
////                        .strafeTo(new Vector2d(-47,13))
////                        //align to sample
////                        .strafeTo(new Vector2d(-47,52))
////                        // push sample
////                        .strafeTo(new Vector2d(-47,13))
////                        //return to position
////                        .strafeTo(new Vector2d(-57, 13))
////                        //align to sample
////                        .strafeTo(new Vector2d(-57, 52))
////                        //push sample
////                        .strafeTo(new Vector2d(-57, 13))
////                        //return to position
////                        .strafeTo(new Vector2d(-61, 13))
////                        //align to sample
////                        .strafeTo(new Vector2d(-61, 52))
////                        //push sample
////                        .strafeTo(new Vector2d(-42,54))
////                        .waitSeconds(.8)
////                        //pick up sample
////                        .setTangent(0)
////                        .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
////                        .setTangent(180)
////                        .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(90)), Math.toRadians(0))
////                        .setTangent(0)
////                        .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
////                        .setTangent(0)
////                        .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(90)), Math.toRadians(0))
//////                        .setTangent(0)
//////                        .splineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(90)), Math.toRadians(0))
////                        .strafeTo(new Vector2d(-42,54))
////                        .setTangent(0)
////                        .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))



        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}