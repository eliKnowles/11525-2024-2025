package org.firstinspires.ftc.teamcode.code;

import static org.firstinspires.ftc.teamcode.code.subsystem.Drive.follower;
import static org.firstinspires.ftc.teamcode.code.subsystem.VSlide.INSTANCE;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.code.paths.Paths;
import org.firstinspires.ftc.teamcode.code.subsystem.HSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Intake;
import org.firstinspires.ftc.teamcode.code.subsystem.VSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Drive;
import org.firstinspires.ftc.teamcode.code.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.hermeshelper.pp.constants.FConstants;
import org.firstinspires.ftc.teamcode.hermeshelper.pp.constants.LConstants;


import com.qualcomm.robotcore.util.ElapsedTime;
import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;


@Mercurial.Attach
@Outtake.Attach
@VSlide.Attach
@HSlide.Attach
@Intake.Attach
@Drive.Attach
@Autonomous(name = "sampleAuto", group = "Spec")
public class sampleAuto extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;



    private final Pose startPose = new Pose(7, 96.000, Math.toRadians(-90));  // Starting position
    private final Pose scorePose1 = new Pose(39.5, 66.000, Math.toRadians(0)); // Scoring position
    private final Pose scorePose2 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private final Pose scorePose3 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private final Pose scorePose4 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private final Pose scorePose5 = new Pose(14, 129, Math.toRadians(0)); // Scoring positio

    private final Pose pickupPose = new Pose(8, 26, Math.toRadians(0)); // First sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position

    private ElapsedTime pathTimer, actionTimer, opmodeTimer;


    private int pathState;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, goAroundFirst, pushSecond, pushThird, scorePickup2, scorePickup3;

    public void buildPaths() {

    }


    public void autonomousPathUpdate() {
    }

    public void setPathState(int pState, boolean resetTimers) {
        pathState = pState;
        if (resetTimers) {
            pathTimer.reset();
            actionTimer.reset();
        } else {
            pathTimer.reset(); // still reset pathTimer by default
        }
    }

    @Override
    public void init() {
        INSTANCE.setDefaultCommand(VSlide.update());
        INSTANCE.setDefaultCommand(HSlide.update());
        HSlide.goTo(0);
        Paths.init();
        Drive.setPose(startPose);
        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        Constants.setConstants(FConstants.class, LConstants.class);
        buildPaths();
    }

    @Override
    public void start() {

        new Sequential(
                new Parallel(
                        Intake.intakeClawOpen(),
                        VSlide.goTo(26000),
                        Outtake.extendArmSample(),
                        Drive.followPathChain(Paths.sample.get(0))
                ),
                Outtake.outtakeClawOpen(),
                Drive.followPathChain(Paths.sample.get(1)),
                Outtake.retractFromBasket(),
                Outtake.retractArmSample(),
                VSlide.goTo(0),
                new Parallel(
                        Intake.runExtend(),
                        HSlide.goTo(13000)
                ),
                new Wait(.2),
                Intake.intakeGrab(),
                new Sequential(
                        Intake.runTransfer(),
                        HSlide.goTo(0),
                        Outtake.outtakeClawClose(),
                        Intake.intakeClawOpen(),
                        new Wait(.2),
                        Intake.intakeSpecimen()),
                new Parallel(
                        VSlide.goTo(26000),
                        Outtake.extendArmSample()
                        ),
                new Wait(.2),
                Drive.followPathChain(Paths.sample.get(2)),
                Outtake.outtakeClawOpen(),
                Drive.followPathChain(Paths.sample.get(3)),
                Outtake.retractFromBasket(),
                Outtake.retractArmSample(),
                VSlide.goTo(0),
                new Parallel(
                        Intake.runExtend(),
                        HSlide.goTo(7000)
                ),
                new Wait(.2),
                Intake.intakeGrab(),
                new Wait(.2),

                new Sequential(
                        Intake.runTransfer(),
                        HSlide.goTo(0),
                        Outtake.outtakeClawClose(),
                        Intake.intakeClawOpen(),
                        new Wait(.2),
                        Intake.intakeSpecimen()),
                new Parallel(
                        VSlide.goTo(26000),
                        Outtake.extendArmSample()
                ),
                Drive.followPathChain(Paths.sample.get(4)),
                Outtake.outtakeClawOpen(),
                Outtake.retractFromBasket(),
                Drive.followPathChain(Paths.sample.get(5)),
                new Parallel(
                        Outtake.retractArmSample(),
                        VSlide.goTo(0) )
               ,
                new Parallel(
                        Intake.runExtend(),
                        HSlide.goTo(7500),
                        Intake.wrist_auto()
                ),
                new Wait(.2),
                Intake.intakeGrab(),
                new Wait(.2),
                new Sequential(
                        Intake.runTransfer(),
                        new Wait(.2),
                        HSlide.goTo(0),
                        Outtake.outtakeClawClose(),
                        Intake.intakeClawOpen(),
                        new Wait(.2),
                        Intake.intakeSpecimen()),
                new Parallel(
                        VSlide.goTo(26000),
                        Outtake.extendArmSample()
                ),
                        Drive.followPathChain(Paths.sample.get(6)),
                Outtake.outtakeClawOpen(),
                Outtake.retractFromBasket(),
                Drive.followPathChain(Paths.sample.get(7)),
                new Parallel(
                        Outtake.retractArmSample(),
                        VSlide.goTo(0))






                //  )



                //  Drive.followPathChain(Paths.sample.get(3)),
                //Outtake.outtakeClawOpen()
              /*  Outtake.retractFromBasket(),
                Outtake.retractArmSample(),
                VSlide.goTo(0),
                new Parallel(
                        Intake.runExtend(),
                        HSlide.goTo(13000)
                ),
                new Wait(.5),
                Intake.intakeGrab() */









                ).schedule();
    }

    @Override
    public void loop() {
        Drive.follower.update();
//        follower.update();
//        autonomousPathUpdate();
        telemetry.addLine(Mercurial.INSTANCE.getActiveCommandSnapshot().toString());
        telemetry.addData("timer", actionTimer);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}