package org.firstinspires.ftc.teamcode.code.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Paths {
    public static ArrayList<PathChain> fiveSpecs = new ArrayList<>();
    public static ArrayList<PathChain> sample = new ArrayList<>();


    public static final PathBuilder builder = new PathBuilder();

    public static final Pose startPose = new Pose(7.600, 66.000, Math.toRadians(0));  // Starting position
    public static final Pose startPoseSample = new Pose(7, 96, Math.toRadians(0));  // Starting position

    private static final Pose scorePose1 = new Pose(39.5, 66.000, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose2 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose3 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose4 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose5 = new Pose(14, 129, Math.toRadians(0)); // Scoring positio

    private final Pose pickupPose = new Pose(8, 26, Math.toRadians(0)); // First sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    public static void init() {
        fiveSpecs.clear();
        sample.clear();


        fiveSpecs.add(new PathBuilder() // score preload 0
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build());

        fiveSpecs.add(new PathBuilder() // go around and push first 1
                .addPath(          new BezierCurve(
                        new Point(39.500, 66.000, Point.CARTESIAN),
                        new Point(1.498, 21.975, Point.CARTESIAN),
                        new Point(112.370, 37.790, Point.CARTESIAN),
                        new Point(29.799, 23.473, Point.CARTESIAN),
                        new Point(92.060, 15.316, Point.CARTESIAN),
                        new Point(16.148, 27.968, Point.CARTESIAN),
                        new Point(8.657, 22.308, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // push seonc 2
                .addPath(   new BezierCurve(
                        new Point(8.657, 22.308, Point.CARTESIAN),
                        new Point(74.414, 29.133, Point.CARTESIAN),
                        new Point(64.758, 9.489, Point.CARTESIAN),
                        new Point(61.262, 15.815, Point.CARTESIAN),
                        new Point(10.321, 13.318, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // push third 3
                .addPath(   new BezierCurve(
                        new Point(10.321, 13.318, Point.CARTESIAN),
                        new Point(65.591, 15.815, Point.CARTESIAN),
                        new Point(88.398, 2.497, Point.CARTESIAN),
                        new Point(10.654, 6.326, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // grab 2nd 4
                .addPath( new BezierLine(
                        new Point(10.654, 6.326, Point.CARTESIAN),
                        new Point(8.000, 28.634, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // score 2nd 5, line 6
                .addPath(new BezierCurve(
                        new Point(8.000, 28.634, Point.CARTESIAN),
                        new Point(9.156, 72.250, Point.CARTESIAN),
                        new Point(41.000, 69.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // go to grab 3rd 6
                .addPath( new BezierCurve(
                        new Point(41.000, 69.000, Point.CARTESIAN),
                        new Point(8.990, 68.587, Point.CARTESIAN),
                        new Point(40.620, 26.802, Point.CARTESIAN),
                        new Point(8.000, 28.634, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // socre 3rd 7
                .addPath(new BezierCurve(
                        new Point(8.000, 28.634, Point.CARTESIAN),
                        new Point(11.320, 75.579, Point.CARTESIAN),
                        new Point(41.000, 72.000, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // grab 4th 8
                .addPath( new BezierCurve(
                        new Point(41.000, 72.000, Point.CARTESIAN),
                        new Point(8.990, 68.587, Point.CARTESIAN),
                        new Point(40.620, 26.802, Point.CARTESIAN),
                        new Point(8.000, 28.634, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());
        fiveSpecs.add(new PathBuilder() // score 4th 9
                .addPath(
                        new BezierCurve(
                                new Point(8.000, 28.634, Point.CARTESIAN),
                                new Point(12.153, 75.246, Point.CARTESIAN),
                                new Point(41.000, 74.000, Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // grab 5th 10
                .addPath(
                        new BezierCurve(
                                new Point(41.000, 74.000, Point.CARTESIAN),
                                new Point(8.990, 68.587, Point.CARTESIAN),
                                new Point(40.620, 26.802, Point.CARTESIAN),
                                new Point(8.000, 28.634, Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // score 5th 11
                .addPath(
                        new BezierCurve(
                                new Point(8.000, 28.634, Point.CARTESIAN),
                                new Point(13.651, 77.910, Point.CARTESIAN),
                                new Point(41.000, 76.000, Point.CARTESIAN)
                        ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // score 5th 11
                .addPath(
                        new BezierCurve(
                                new Point(41.000, 76.000, Point.CARTESIAN),
                                new Point(16.481, 68.754, Point.CARTESIAN),
                                new Point(12.818, 15.471, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(45))
                .build());

        sample.add(new PathBuilder() // score first
                .addPath(
                        new BezierLine(
                                new Point(7.000, 96.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45))
                .build());
        sample.add(new PathBuilder() // grab position 1
                .addPath(
                        // Line
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(16.000, 123.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build());

        sample.add(new PathBuilder() // score second  2
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(16.000, 123.000, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build());
        sample.add(new PathBuilder() // grab position second 3
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(24.000, 131.750, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build());
        sample.add(new PathBuilder() // score 2nd
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(24.000, 131.750, Point.CARTESIAN),
                                new Point(13.000, 130.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build());

        sample.add(new PathBuilder() // grab position 3
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(13.000, 130.000, Point.CARTESIAN),
                                new Point(18.978, 118.363, Point.CARTESIAN),
                                new Point(29.133, 125.521, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45))
                .build());
        sample.add(new PathBuilder() // score 3
                .addPath(
                        new BezierLine(
                                new Point(29.133, 125.521, Point.CARTESIAN),
                                new Point(15.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-45))
                .build());
        sample.add(new PathBuilder() // score 3
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(15.000, 129.000, Point.CARTESIAN),
                                new Point(56.934, 129.683, Point.CARTESIAN),
                                new Point(61.762, 96.555, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90))
                .build());
    }

    public static void build(Pose currentPose) {}

    public static Path pathTo(BezierLine line, Follower follower) {
        return createPath(
                new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY()),
                        new Point(line.getLastControlPoint().getX(), line.getLastControlPoint().getY())
                )
        );
    }

    public static Path pathTo(Point end, Pose current) {
        return createPath(
                new BezierLine(
                        new Point(current), end
                ), current.getHeading()
        );
    }

    public static Path curveTo(BezierCurve curve, Follower follower) {
        ArrayList<Point> controlPoints = curve.getControlPoints();
        controlPoints.remove(0);

        List<Point> points = new ArrayList<>();
        points.add(new Point(follower.getPose().getX(), follower.getPose().getY()));
        points.addAll(controlPoints);

        return createPath(new BezierCurve(points.toArray(new Point[0])));
    }

    private static Path createPath(BezierLine line, double heading) {
        Path path = new Path(line);
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    private static Path createPath(BezierLine line, double startHeading, double endHeading) {
        Path path = new Path(line);
        path.setLinearHeadingInterpolation(startHeading, endHeading);
        return path;
    }

    private static Path createPath(BezierLine line, boolean tangent) {
        Path path = new Path(line);
        if (tangent) path.setTangentHeadingInterpolation();
        return path;
    }

    private static Path createPath(BezierLine line) {
        Path path = new Path(line);
        path.setConstantHeadingInterpolation(0);
        return path;
    }

    private static Path createPath(BezierCurve curve) {
        Path path = new Path(curve);
        path.setConstantHeadingInterpolation(0);
        return path;
    }

    private static Path createPath(BezierCurve curve, double heading) {
        Path path = new Path(curve);
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    private static Path createPath(BezierCurve curve, double startHeading, double endHeading) {
        Path path = new Path(curve);
        path.setLinearHeadingInterpolation(startHeading, endHeading);
        return path;
    }

    private static Path createPath(BezierCurve curve, boolean tangent) {
        Path path = new Path(curve);
        if (tangent) path.setTangentHeadingInterpolation();
        else path.setConstantHeadingInterpolation(0);
        return path;
    }
}
