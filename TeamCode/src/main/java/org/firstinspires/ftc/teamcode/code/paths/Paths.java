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

    private static final Pose startPose = new Pose(7.600, 66.000, Math.toRadians(0));  // Starting position
    private static final Pose scorePose1 = new Pose(39.5, 66.000, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose2 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose3 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose4 = new Pose(14, 129, Math.toRadians(0)); // Scoring position
    private static final Pose scorePose5 = new Pose(14, 129, Math.toRadians(0)); // Scoring positio

    private final Pose pickupPose = new Pose(8, 26, Math.toRadians(0)); // First sample pickup

    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position

    public static void init() {
        Collections.addAll(fiveSpecs,
                builder
                        .addPath(new BezierLine(new Point(startPose), new Point(scorePose1)))
                        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
                        .build(),
                builder
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
                        .build(),
                builder
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

                        .build(),
                builder
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
                        .build(),
                builder
                        .addPath(
                                // Line 5
                                new BezierLine(
                                        new Point(12.486, 6.825, Point.CARTESIAN),
                                        new Point(7.000, 25.000, Point.CARTESIAN) // grab spec
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build(),
                builder
                        .addPath(
                                // Line 6
                                new BezierCurve(
                                        new Point(8.000, 25.000, Point.CARTESIAN),
                                        new Point(7.658, 66.423, Point.CARTESIAN),
                                        new Point(40.500, 69.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build(),
                builder
                        .addPath(
                                // Line 7
                                new BezierLine(
                                        new Point(40.500, 69.000, Point.CARTESIAN),
                                        new Point(7.000, 35.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build(),
                builder
                        .addPath(
                                // Line 8
                                new BezierLine(
                                        new Point(7.000, 35.000, Point.CARTESIAN),
                                        new Point(41.000, 72.000, Point.CARTESIAN)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                        .build()
        );
    }

    public static void build(Pose currentPose) {

    }

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

    // Constant Heading
    private static Path createPath(BezierCurve curve, double heading) {
        Path path = new Path(curve);
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    // Linear Heading
    private static Path createPath(BezierCurve curve, double startHeading, double endHeading) {
        Path path = new Path(curve);
        path.setLinearHeadingInterpolation(startHeading, endHeading);
        return path;
    }

    // Tangent or Default Constant Heading
    private static Path createPath(BezierCurve curve, boolean tangent) {
        Path path = new Path(curve);
        if (tangent) path.setTangentHeadingInterpolation();
        else path.setConstantHeadingInterpolation(0);
        return path;
    }
}
