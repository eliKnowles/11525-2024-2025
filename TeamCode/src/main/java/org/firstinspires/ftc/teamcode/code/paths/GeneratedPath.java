package org.firstinspires.ftc.teamcode.code.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.VSlide;

import dev.frozenmilk.mercurial.commands.groups.Sequential;

public class GeneratedPath {
    public static PathBuilder builder = new PathBuilder();

    public GeneratedPath() {
        PathChain path = new PathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.600, 66.000, Point.CARTESIAN),
                                new Point(41.000, 66.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setReversed(true)
                .addTemporalCallback(100, () -> {

                })
                .build();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.600, 66.000, Point.CARTESIAN),
                                new Point(41.000, 66.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setReversed(true)
                

                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(41.000, 66.000, Point.CARTESIAN),
                                new Point(30.964, 31.297, Point.CARTESIAN),
                                new Point(50.608, 33.295, Point.CARTESIAN),
                                new Point(66.257, 28.634, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setReversed(true)
                

                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(66.257, 28.634, Point.CARTESIAN),
                                new Point(72.416, 21.142, Point.CARTESIAN),
                                new Point(14.150, 22.474, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setReversed(true)
                

                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(14.150, 22.474, Point.CARTESIAN),
                                new Point(54.271, 28.634, Point.CARTESIAN),
                                new Point(63.760, 13.151, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(63.760, 13.151, Point.CARTESIAN),
                                new Point(13.651, 12.818, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(13.651, 12.818, Point.CARTESIAN),
                                new Point(57.434, 11.653, Point.CARTESIAN),
                                new Point(72.583, 17.313, Point.CARTESIAN),
                                new Point(57.267, 5.993, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(57.267, 5.993, Point.CARTESIAN),
                                new Point(13.651, 5.494, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(13.651, 5.494, Point.CARTESIAN),
                                new Point(8.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(8.000, 26.000, Point.CARTESIAN),
                                new Point(41.000, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(41.000, 69.000, Point.CARTESIAN),
                                new Point(8.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(8.000, 26.000, Point.CARTESIAN),
                                new Point(41.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(41.000, 72.000, Point.CARTESIAN),
                                new Point(8.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(8.000, 26.000, Point.CARTESIAN),
                                new Point(41.000, 76.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(41.000, 76.000, Point.CARTESIAN),
                                new Point(8.000, 26.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                

                .addPath(
                        // Line 15
                        new BezierLine(
                                new Point(8.000, 26.000, Point.CARTESIAN),
                                new Point(41.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0));
    }

    public PathBuilder getBuilder() {
        return builder;
    }
}

