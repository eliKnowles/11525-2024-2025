package org.firstinspires.ftc.teamcode.code.limelight;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.pathgen.Vector;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Optional;

import dev.frozenmilk.mercurial.commands.Lambda;

public class LimelightCV {
    public static final PathBuilder PATH_BUILDER = new PathBuilder();

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

    public void align(Sample sample) {
        double height = 0.25;
        double targetHeight = 0.03;
        double mountAngle = 25; // TODO: lucas change this please

        double ty = sample.getTy();
        double tx = sample.getTx();

        double distance = getDistance(ty, height, targetHeight, mountAngle);
        double angleRad = Math.toRadians(tx);

        double dx = distance * Math.cos(angleRad);
        double dy = distance * Math.sin(angleRad);

        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading();

        double offsetX = dx * Math.cos(heading) - dy * Math.sin(heading);
        double offsetY = dx * Math.sin(heading) + dy * Math.cos(heading);

        double targetX = currentPose.getX() + offsetX;
        double targetY = currentPose.getY() + offsetY;

        Pose targetPose = new Pose(targetX, targetY, heading);

        follower.followPath(
                PATH_BUILDER.addBezierLine(
                        new Point(currentPose),
                        new Point(targetPose)
                ).build()
        );
    }

    public void align() {
        align(scan().orElseGet(() -> {
            System.out.println("Could not detect sample in latest scan");
            return new Sample(0, 0);
        }));
    }

    public Lambda alignAction(Sample sample) {
        return new Lambda("align with sample")
                .setExecute(() -> align(sample))
                .setFinish(follower::isBusy);
    }

    public Lambda alignAction() {
        return new Lambda("align with sample")
                .setExecute(() -> align(
                        scan().orElseGet(() -> {
                            System.out.println("Could not detect sample in latest scan");
                            return new Sample(0, 0);
                        }
                )))
                .setFinish(follower::isBusy);
    }

    double getDistance(double ty, double height, double targetHeight, double angle) {
        double angleTarget = angle + ty;
        double angleRad = Math.toRadians(angleTarget);
        double heightDiff = targetHeight - height;
        return heightDiff / Math.tan(angleRad);
    }
}
