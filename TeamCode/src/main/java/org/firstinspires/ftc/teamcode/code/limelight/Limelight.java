package org.firstinspires.ftc.teamcode.code.limelight;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.code.subsystem.HSlide;
import org.firstinspires.ftc.teamcode.code.subsystem.Intake;

public class Limelight {
    private final Limelight3A hardware;

    private double position = 0;

    public Limelight(HardwareMap hwmp) {
        this.hardware = hwmp.get(Limelight3A.class, "limelight");
    }

    public static class SampleState {
        public double angle;
        public Vector center;

        public Vector robotPosition;
        public double robotRotation;

        public double slidePosition;

        public SampleState(double angle, Vector center, Vector robotPosition, double robotRotation, double slidePosition) {
            this.angle = angle;
            this.center = center;
            this.robotPosition = robotPosition;
            this.robotRotation = robotRotation;
            this.slidePosition = slidePosition;
        }

        public SampleState() {
            this.angle = 0;
            this.center = Vector.cartesian(0, 0);
            this.robotPosition = Vector.cartesian(0, 0);
            this.robotRotation = 0;
            this.slidePosition = 0;
        }
    }

//    public SampleState query(Telemetry telemetry, Follower follower) {
//        LLResult result = hardware.getLatestResult();
//
//        telemetry.addData("SOMERESULT", result == null);
//        if (result == null) return null;
//
//        double[] result_array = result.getPythonOutput();
//        telemetry.addData("SOMEPYTHON", result_array == null);
//
//        if (result_array == null) return null;
//        if (result_array.length == 0) return null;
//        double angle = result_array[0];
//
//        Vector center = Vector.cartesian(result_array[1], result_array[2]);
//        Pose current = follower.getPose();
//        return new SampleState(angle, center, Vector.cartesian(current.getX(),
//                current.getY()), current.getHeading(), HSlide.getPosition());
//    }

    public SampleState query(Follower follower) {
        LLResult result = hardware.getLatestResult();

        double angle = result.getTa();

        Vector center = Vector.cartesian(result.getTx(), result.getTy());
        Pose current = follower.getPose();
        return new SampleState(angle, center, Vector.cartesian(current.getX(),
                current.getY()), current.getHeading(), HSlide.getPosition());
    }
}