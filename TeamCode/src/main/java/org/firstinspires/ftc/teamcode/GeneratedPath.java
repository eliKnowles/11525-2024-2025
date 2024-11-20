package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class GeneratedPath {
    private final PathChain path;

    public GeneratedPath() {
        PathBuilder builder = new PathBuilder();

        path = builder
                .addPath(
                        new BezierCurve(
                                new Point(9.757, 84.983, Point.CARTESIAN),
                                new Point(24.000, 137.739, Point.CARTESIAN),
                                new Point(57.391, 124.435, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .build(); // Ensure you call build() to finalize the path
    }

    public PathChain getPath() {
        return path;
    }
}
