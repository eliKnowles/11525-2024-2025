package org.firstinspires.ftc.teamcode.hermeshelper.tuning;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hermeshelper.rr.PinpointDrive;

public final class SplineTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(-23.5, 63,Math.toRadians(90));
//        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
            waitForStart();
            Actions.runBlocking(
                    drive.actionBuilder(beginPose)

                            // place 1st specimen
                            .setReversed(true)
                            .splineTo(new Vector2d(-5, 36), Math.toRadians(270))
                            // once at -5, 36, open claw

                            //pivotservo set to specimen
                            //slides set to specimen
                            .waitSeconds(.2)
                            //action

                            // drive to push samples
                            .setReversed(false)
                            .splineTo(new Vector2d(-34, 28), Math.toRadians(270)) //waypoint to first sample

                            .splineTo(new Vector2d(-45, 6), Math.toRadians(270)) // align with first sample


                            .setReversed(true)
                            .splineTo(new Vector2d(-46,53), Math.toRadians(90))// push first sample
                            .setReversed(false)// push first sample

                            .splineTo(new Vector2d(-47, 4), Math.toRadians(270))  // return
                            .setReversed(true)
                            .splineTo(new Vector2d(-56, 22), Math.toRadians(90)) // waypoint
                            .splineTo(new Vector2d( -57, 53), Math.toRadians(90))  //push 2nd
                            .setReversed(false)


                            //cycle 2nd specimen
                            .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
                            //grab 2nd
                            .waitSeconds(.2)
                            .setTangent(Math.toRadians(90))
                            .splineTo(new Vector2d(-37, 57), Math.toRadians(90))

                            //place 2nd
                            .setTangent(Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
                            .waitSeconds(.15)

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
                            .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                        .waitSeconds(.1)
//







                            .build());
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-5, 36), Math.toRadians(270))
//                            .waitSeconds(.2)
//                            //action
//
//                            .setReversed(false)
//                            .splineTo(new Vector2d(-35, 28), Math.toRadians(270))
//
//                            .splineTo(new Vector2d(-45, 6), Math.toRadians(270)) // align with first sample
//
//
////                            .strafeTo(new Vector2d(-45,53)) // push first sample
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-45,53), Math.toRadians(90))
//                            .setReversed(false)// push first sample
//
//                            .splineTo(new Vector2d(-47, 11), Math.toRadians(270))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-55, 25), Math.toRadians(90))
//                            .splineTo(new Vector2d( -55, 53), Math.toRadians(90))
//                            .setReversed(false)
////
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-56, 25), Math.toRadians(90)) //waypoint
//
//                            .splineTo(new Vector2d( -56, 53), Math.toRadians(90)) //pushed
////                            .setReversed(false)
////                        .splineTo(new Vector2d( -57, 11), Math.toRadians(270)) //return for last
////                            .setReversed(true)
////                            .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
////
//                            .setReversed(false)
////                            .waitSeconds(.3)
//                            .strafeTo(new Vector2d(-37, 57)) //intake
//
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                            .waitSeconds(.2)
//
//                            .setReversed(true)
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//                            .setReversed(false)
//                            .strafeTo(new Vector2d(-37, 57))
//                            .waitSeconds(.2) // grab
//
//
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                            .waitSeconds(.2)
//
//
//                            .setReversed(true)
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//                            .setReversed(false)
//                            .strafeTo(new Vector2d(-37, 57))
//                            .waitSeconds(.3) // grab
//
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                            .waitSeconds(.3)
//
//                            .setReversed(true)
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d( -37, 50, Math.toRadians(270)), Math.toRadians(90))//position for intaking
//                            .setReversed(false)
//                            .strafeTo(new Vector2d(-37, 57))
//                            .waitSeconds(.1) // grab
//
//                            .setTangent(Math.toRadians(0))
//                            .splineToLinearHeading(new Pose2d(-2, 36, Math.toRadians(90)), Math.toRadians(270))
//                        .waitSeconds(.1)





//                        .strafeTo(new Vector2d(-61,11))  //align with last sample

//

//
//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 60), Math.PI)
//                            .build());
//        } else {
//            throw new RuntimeException();
//        }
        waitForStart();
        }
    }
