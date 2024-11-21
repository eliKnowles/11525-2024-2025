package org.firstinspires.ftc.teamcode.tuning;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.hermeshelper.util.Sequence;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    Sequence sequence = new Sequence();
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-26, 63,4.71239);
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(-36, 13), 4.71239)
                            .strafeTo(new Vector2d(-47,13))
                            //align to sample
                            .strafeTo(new Vector2d(-47,52))
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


                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
                            .setTangent(180)
                            .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(270)), Math.toRadians(0))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-5, 37, Math.toRadians(90)), Math.toRadians(0))
                            .setTangent(0)
                            .splineToLinearHeading(new Pose2d(-42, 54, Math.toRadians(270)), Math.toRadians(0))
//                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
//        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
//            TankDrive drive = new TankDrive(hardwareMap, beginPose);
//
            waitForStart();
//
//            Actions.runBlocking(
//                    drive.actionBuilder(beginPose)
//                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            .splineTo(new Vector2d(0, 60), Math.PI)
//                            .build());
//        } else {
//            throw new RuntimeException();
//        }
        }
    }
}