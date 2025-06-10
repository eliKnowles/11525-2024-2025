package org.firstinspires.ftc.teamcode.code;

import static org.firstinspires.ftc.teamcode.code.subsystem.Drive.follower;
import static org.firstinspires.ftc.teamcode.code.subsystem.VSlide.INSTANCE;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.code.limelight.Limelight;
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
@Autonomous(name = "Five Spec no cv", group = "Spec")
public class five_spec_no_cv extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;

    private Limelight limelight;
    private Limelight.SampleState buffer;

    private final Pose startPose = new Pose(7.600, 66.000, Math.toRadians(0));  // Starting position
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
    private double clawTimeout = .1;
    private double retractTimeout = 0.01;

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
        this.buffer = new Limelight.SampleState();
        this.limelight = new Limelight(hardwareMap);
        this.pathTimer = new ElapsedTime();
        this.opmodeTimer = new ElapsedTime();
        this.actionTimer = new ElapsedTime();

        INSTANCE.setDefaultCommand(VSlide.update());
        INSTANCE.setDefaultCommand(HSlide.update());

        HSlide.goTo(0);
        Paths.init();
        Drive.setPose(startPose);
        Constants.setConstants(FConstants.class, LConstants.class);
        buildPaths();
    }

    @Override
    public void start() {

        new Sequential(
                new Parallel(
                        VSlide.goTo(19000, 1.0),
                        Intake.intakeSpecimen(),
                        Outtake.scoreSpecimenAuto(),
                        new Sequential(
                                new Wait(.2),
                                Drive.followPathChain(Paths.fiveSpecs.get(0))
                )
                ), // score preload,


                Outtake.retractFromChamber(),
                new Wait(retractTimeout),
                new Parallel(
                        Drive.followPathChain(Paths.fiveSpecs.get(1)),   // go around and push first
                        Outtake.grabSpecimen(),
                        VSlide.goTo(0, .4)
               ),
                new Sequential(
                        Drive.followPathChain(Paths.fiveSpecs.get(2)), // push second
                        Drive.followPathChain(Paths.fiveSpecs.get(3)),// push third
                        Drive.followPathChain(Paths.fiveSpecs.get(4)),
                        Outtake.outtakeClawClose(),
                        new Wait(clawTimeout)
                        ),
                new Parallel(
                        Outtake.scoreSpecimen(),
                        VSlide.goTo(19000),
                        Drive.followPathChain(Paths.fiveSpecs.get(5))
                ),
                Outtake.retractFromChamber(),
                new Wait(retractTimeout),
                new Parallel(
                        Outtake.grabSpecimen(),
                        VSlide.goTo(0,.4),
                        Drive.followPathChain(Paths.fiveSpecs.get(6))
                ),
                new Wait(clawTimeout),
                Outtake.outtakeClawClose(),
                new Parallel(
                        Outtake.scoreSpecimen(), // score 3rd
                        VSlide.goTo(19000),
                        Drive.followPathChain(Paths.fiveSpecs.get(7)) // drive to score 3rd
                ),
                Outtake.retractFromChamber(),
                new Wait(retractTimeout),
                new Parallel(
                        Outtake.grabSpecimen(),
                        VSlide.goTo(0,.4),
                        Drive.followPathChain(Paths.fiveSpecs.get(8)) // grab 4th
                ),
                new Wait(clawTimeout),
                Outtake.outtakeClawClose(),
                new Parallel(
                        Outtake.scoreSpecimen(), // score 4th
                        VSlide.goTo(19000),
                        Drive.followPathChain(Paths.fiveSpecs.get(9)) // drive to score 4th
                ),
                Outtake.retractFromChamber(),
               new Wait(retractTimeout),

                new Parallel(
                        Outtake.grabSpecimen(),
                        VSlide.goTo(0,1),
                        Drive.followPathChain(Paths.fiveSpecs.get(10))),
                new Wait(clawTimeout),
                Outtake.outtakeClawClose(),
                new Parallel(
                        Outtake.scoreSpecimen(),
                        VSlide.goTo(19000),
                        Drive.followPathChain(Paths.fiveSpecs.get(11)) // drive to score 5th
                ),
                Outtake.retractFromChamber(),
                new Wait(retractTimeout),
                VSlide.goTo(0,.4),
                Drive.followPathChain(Paths.fiveSpecs.get(12))


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