package org.firstinspires.ftc.teamcode.code.limelight;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.code.subsystem.HSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Intake;

import java.util.Collections;
import java.util.Set;

import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;

public class ScanForSample implements Command {
    private Limelight limelight;
    private Limelight.SampleState result;

    private Follower follower;

    private boolean check;
    private double tilt = 0;


    public ScanForSample(Limelight limelight, Limelight.SampleState buffer, Follower follower, boolean isSub) {
        this.limelight = limelight;
        this.result = buffer;
        this.follower = follower;
        this.check = isSub;
    }

    public ScanForSample tilt(double newTilt) {
        this.tilt = newTilt;
        return this;
    }

    @Override
    public void initialise() {
    }

    @Override
    public void execute() {
        Limelight.SampleState detection = limelight.query(follower);

        if (detection != null) {

            double x = detection.robotPosition.x;
            double y = detection.robotPosition.y;
            double r = detection.robotRotation;

            double tx = 1 - 0.8 * detection.center.x;
            double ty = 2.7 - Math.pow(detection.center.y, 2);

            double relativeX = ty * Math.cos(r) + tx * Math.cos(r - Math.toRadians(90));
            double relativeY = ty * Math.sin(r) + tx * Math.sin(r - Math.toRadians(90));

            double targetX = x + relativeX;
            double targetY = y + relativeY;

            if (targetY < -16 && check) return;
            if (targetX < 49 && check) return;

            this.result.angle = detection.angle;
            this.result.center = detection.center;
            this.result.robotPosition = detection.robotPosition;
            this.result.robotRotation = detection.robotRotation;
            this.result.slidePosition = HSlide.getPosition();
        } else {
        }
    }
    @Override
    public boolean finished() {
        return this.result.angle != 0 && this.result.angle != 90;
    }

    @Override
    public void end(boolean i) {
    }

    @NonNull
    @Override
    public Set<Object> getRequirements() {
        return Collections.emptySet();
    }

    @NonNull
    @Override
    public Set<Wrapper.OpModeState> getRunStates() {
        return Collections.emptySet();
    }
}