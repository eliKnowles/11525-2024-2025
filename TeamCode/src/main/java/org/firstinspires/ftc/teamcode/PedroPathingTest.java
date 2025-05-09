package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

@Config
public class PedroPathingTest extends CommandOpMode {
    private Follower follower;

    @Override
    public void initialize() {
        follower = new Follower(hardwareMap, (Localizer) telemetry);
        follower.setStartingPose(new Pose(9, 111, Math.toRadians(270)));

        Pose startPose = new Pose(9, 111, Math.toRadians(270));
        Pose scorePose = new Pose(14, 129, Math.toRadians(315));

        Path toScore = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        toScore.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        PathChain scorePreloadChain = follower.pathBuilder()
                // both work like async task for rr
                .addPath(toScore)
                .addParametricCallback(0.8, () -> {// executes when robot gets to spot
                    telemetry.addLine("test1");
                    telemetry.update();
                })
                .addTemporalCallback(1000, () -> {// after (time) ms since start of path
                    telemetry.addLine("test2");
                    telemetry.update();
                })
                .build();

        // do da thing
        follower.followPath(scorePreloadChain);
    }

    @Override
    public void run() {
        follower.update();
    }
}
