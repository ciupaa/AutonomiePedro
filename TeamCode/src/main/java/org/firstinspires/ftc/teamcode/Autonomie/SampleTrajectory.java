package org.firstinspires.ftc.teamcode.Autonomie;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;

public class SampleTrajectory {
    public class GeneratedPath {

        public GeneratedPath() {
            PathBuilder builder = new PathBuilder();

            builder
                    .addPath(
                            // Line 1
                            new BezierCurve(
                                    new Point(9.000, 85.000, Point.CARTESIAN),
                                    new Point(34.766, 97.570, Point.CARTESIAN),
                                    new Point(18.168, 124.486, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .addPath(
                            // Line 2
                            new BezierLine(
                                    new Point(18.168, 124.486, Point.CARTESIAN),
                                    new Point(30.729, 121.346, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierLine(
                                    new Point(30.729, 121.346, Point.CARTESIAN),
                                    new Point(18.168, 124.486, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .addPath(
                            // Line 4
                            new BezierCurve(
                                    new Point(18.168, 124.486, Point.CARTESIAN),
                                    new Point(21.757, 134.804, Point.CARTESIAN),
                                    new Point(30.729, 131.215, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                    .addPath(
                            // Line 5
                            new BezierCurve(
                                    new Point(30.729, 131.215, Point.CARTESIAN),
                                    new Point(30.953, 120.000, Point.CARTESIAN),
                                    new Point(18.168, 124.486, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                    .addPath(
                            // Line 6
                            new BezierCurve(
                                    new Point(18.168, 124.486, Point.CARTESIAN),
                                    new Point(41.720, 113.720, Point.CARTESIAN),
                                    new Point(45.308, 129.645, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                    .addPath(
                            // Line 7
                            new BezierCurve(
                                    new Point(45.308, 129.645, Point.CARTESIAN),
                                    new Point(41.720, 113.495, Point.CARTESIAN),
                                    new Point(18.168, 124.486, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .addPath(
                            // Line 8
                            new BezierCurve(
                                    new Point(18.168, 124.486, Point.CARTESIAN),
                                    new Point(55.850, 124.935, Point.CARTESIAN),
                                    new Point(57.421, 95.551, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90));
        }
    }

}
