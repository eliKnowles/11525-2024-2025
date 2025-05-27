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

    public static final PathBuilder builder = new PathBuilder();

    public static final Pose startPose = new Pose(7.600, 66.000, Math.toRadians(0));  // Starting position
    private static final Pose scorePose1 = new Pose(39.5, 66.000, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose2 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose3 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose4 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose5 = new Pose(14, 129, Math.toRadians(0)); // Scoring positio

    private final Pose pickupPose = new Pose(8, 26, Math.toRadians(0)); // First sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
    public static void init() {
        fiveSpecs.clear();

        fiveSpecs.add(new PathBuilder() // score preload 0
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                .build());

        fiveSpecs.add(new PathBuilder() // go around and push first 1
                .addPath( new BezierCurve(
                        new Point(39.500, 66.000, Point.CARTESIAN),
                        new Point(4.162, 10.821, Point.CARTESIAN),
                        new Point(113.036, 45.947, Point.CARTESIAN),
                        new Point(73.914, 18.312, Point.CARTESIAN),
                        new Point(13.984, 23.306, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // push seonc 2
                .addPath( new BezierCurve(
                        new Point(8.657, 22.308, Point.CARTESIAN),
                        new Point(76.412, 28.800, Point.CARTESIAN),
                        new Point(64.758, 9.489, Point.CARTESIAN),
                        new Point(61.262, 15.815, Point.CARTESIAN),
                        new Point(10.321, 13.318, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build());

        fiveSpecs.add(new PathBuilder() // push third 3
                .addPath(  new BezierCurve(
                        new Point(10.321, 13.318, Point.CARTESIAN),
                        new Point(65.591, 16.647, Point.CARTESIAN),
                        new Point(91.228, 2.664, Point.CARTESIAN),
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
                        new Point(40.000, 69.000, Point.CARTESIAN)
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
