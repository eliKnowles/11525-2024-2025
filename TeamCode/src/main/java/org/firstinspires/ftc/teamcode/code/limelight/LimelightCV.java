package org.firstinspires.ftc.teamcode.code.limelight;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.code.subsystem.Drive;

import java.util.Optional;

import dev.frozenmilk.mercurial.commands.Lambda;

@Drive.Attach
public class LimelightCV {
    public final Limelight3A limelight;
    public final Follower follower;

    public LimelightCV(HardwareMap hwmp, Follower follower) {
        this.limelight = hwmp.get(Limelight3A.class, "limelight");
        this.follower = follower;

        limelight.start();
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
    }

    public Optional<Sample> scan() {
        LLResult result = limelight.getLatestResult();
        if (!result.isValid()) {
            return Optional.empty();
        }

        double tx = result.getTx();
        double ty = result.getTy();

        return Optional.of(new Sample(tx, ty));
    }

    public PathChain align(Sample sample) {
        double tx = sample.getTx();
        double ty = sample.getTy();

        Pose currentPose = follower.getPose();

        double xScale = 0.02;
        double yScale = 0.2;

        double targetX = currentPose.getX() + (ty * yScale);
        double targetY = currentPose.getY() - (tx * xScale);

        double heading = currentPose.getHeading();

        Point targetPose = new Point(targetX, targetY, Point.CARTESIAN);

        PathBuilder pathBuilder = new PathBuilder();

        boolean reversed = ty < 0;

        PathChain chain = new PathBuilder()
                .addPath(new BezierLine(
                        new Point(currentPose.getX(), currentPose.getY(), Point.CARTESIAN),
                        targetPose
                ))
                .setConstantHeadingInterpolation(heading)
                .setReversed(reversed)
                .build();

        return chain;
    }

    public PathChain align() {
        return align(scan().orElseGet(() -> {
            throw new RuntimeException("Could not detect sample in latest scan");
//            return new Sample(0, 0);
        }));
    }

    public Lambda alignAction(Sample sample) {
        return new Lambda("align with sample")
                .setExecute(() -> align(sample))
                .setFinish(() -> true);
    }

    public Lambda alignAction() {
        return new Lambda("align with sample")
                .setInit(this::align)
//                .setExecute(follower::update)
                .setFinish(follower::isBusy);
    }

    double getDistance(double ty, double height, double targetHeight, double angle) {
        double angleTarget = angle + ty;
        double angleRad = Math.toRadians(angleTarget);
        double heightDiff = targetHeight - height;
        return heightDiff / Math.tan(angleRad);
    }
}
