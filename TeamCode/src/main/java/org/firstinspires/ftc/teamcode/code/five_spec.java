package org.firstinspires.ftc.teamcode.code;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.VSlide;
import org.firstinspires.ftc.teamcode.code.util.Outtake;
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
@Intake.Attach
@Autonomous(name = "Five Spec", group = "Spec")
public class five_spec extends OpMode {
    private DashboardPoseTracker dashboardPoseTracker;


    private final Pose startPose = new Pose(7.600, 66.000, Math.toRadians(0));  // Starting position
    private final Pose scorePose1 = new Pose(39.5, 66.000, Math.toRadians(0)); // Scoring position
    private final Pose scorePose2 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private final Pose scorePose3 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private final Pose scorePose4 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private final Pose scorePose5 = new Pose(14, 129, Math.toRadians(0)); // Scoring positio

    private final Pose pickupPose = new Pose(8, 26, Math.toRadians(0)); // First sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position

    private Follower follower;


    private ElapsedTime pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, goAroundFirst, pushSecond, pushThird, scorePickup2, scorePickup3;

    public void buildPaths() {
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .addParametricCallback(0.01, () -> {
                    VSlide.goTo(18700).schedule(); //score preload
                })



                .build();

        goAroundFirst = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(41.000, 66.000, Point.CARTESIAN),
                                new Point(14.483, 57.101, Point.CARTESIAN),
                                new Point(33.128, 38.622, Point.CARTESIAN),
                                new Point(88.231, 11.653, Point.CARTESIAN),
                                new Point(73.748, 25.970, Point.CARTESIAN),
                                new Point(12.319, 21.642, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addParametricCallback(0.01, () -> {
                    Outtake.setOuttakeClaw(Outtake.outtakeClawPosition.OPEN.pos);
                })

                .addParametricCallback(0.2, () -> {
                    new Sequential(
                            VSlide.goTo(0),
                            Outtake.grabSpecimen()
                    ).schedule();
                })

                .build();

        pushSecond = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(12.319, 21.642, Point.CARTESIAN),
                                new Point(90.728, 23.972, Point.CARTESIAN),
                                new Point(54.603, 10.321, Point.CARTESIAN),
                                new Point(49.443, 12.486, Point.CARTESIAN),
                                new Point(12.818, 13.151, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                .build();
        pushThird = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(12.818, 13.151, Point.CARTESIAN),
                                new Point(94.557, 10.821, Point.CARTESIAN),
                                new Point(47.778, 5.827, Point.CARTESIAN),
                                new Point(65.424, 6.492, Point.CARTESIAN),
                                new Point(12.486, 6.825, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(12.486, 6.825, Point.CARTESIAN),
                                new Point(7.000, 25.000, Point.CARTESIAN) // grab spec
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addParametricCallback(.999999, () -> {
                    new Sequential(new Wait(.2), Outtake.outtakeClawClose()).schedule();
               })


                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(8.000, 25.000, Point.CARTESIAN),
                                new Point(7.658, 66.423, Point.CARTESIAN),
                                new Point(40.500, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addParametricCallback(0, () -> {
                    new Parallel(VSlide.goTo(18700), Outtake.scoreSpecimen()).schedule();

                })


                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(40.500, 69.000, Point.CARTESIAN),
                                new Point(7.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addParametricCallback(.001, () -> {
                    Outtake.outtakeClawOpen().schedule();


                })
                .addParametricCallback(.1, () -> {
                    new Parallel( Outtake.grabSpecimen(), VSlide.goTo(0, 1)).schedule();

                })
                .addParametricCallback(.9999, () -> {
                    new Sequential(new Wait(.2), Outtake.outtakeClawClose()).schedule();

                })
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(7.000, 35.000, Point.CARTESIAN),
                                new Point(41.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addParametricCallback(.05, () -> {
                    new Parallel(VSlide.goTo(18700), Outtake.scoreSpecimen()).schedule();

                })
                .build();
    }





    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePickup1, true);
                setPathState(1, false);
                break;

            case 1:
                if (!follower.isBusy()) { // <-- THIS IS CRITICAL
                    follower.followPath(goAroundFirst, true);
                    setPathState(2, false);
                }
                break;
            case 2:
                if (!follower.isBusy()) { // <-- THIS IS CRITICAL
                    follower.followPath(pushSecond, true);
                    setPathState(3, false);
                }
                break;
            case 3:
               if (!follower.isBusy()) { // <-- THIS IS CRITICAL
                    follower.followPath(pushThird, true);
                   setPathState(4, false);
               }
               break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true); //
                    setPathState(5, true);                  //
                }
                break;

            case 5:
                if (!follower.isBusy() && actionTimer.seconds() > 3.0) {
                    follower.followPath(scorePickup1, true);
                    setPathState(6, true);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(7, true);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    actionTimer.reset();
                    setPathState(71, false);
                }
                break;

            case 71:
                if (actionTimer.seconds() > 3.0) {
                    follower.followPath(scorePickup2, true);
                    setPathState(8, true);
                }
                break;





        }
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
        VSlide.INSTANCE.setDefaultCommand(VSlide.update());

        pathTimer = new ElapsedTime();
        opmodeTimer = new ElapsedTime();
        actionTimer = new ElapsedTime();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void start() {
        opmodeTimer.reset();
        actionTimer.reset();
            Outtake.scoreSpecimenAuto().schedule();
            // already a Parallel
            setPathState(0, false);
    }

    @Override
    public void loop() {

        follower.update();
        autonomousPathUpdate();
        telemetry.addData("timer", actionTimer);
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}