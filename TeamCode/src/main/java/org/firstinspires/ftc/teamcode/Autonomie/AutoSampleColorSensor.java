package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.SubSystems.ServoRotire;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@Config
@Autonomous(name = "AutoSampleColorSensor")


public class AutoSampleColorSensor extends PedroOpMode {
    public AutoSampleColorSensor() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park, Middle, StrafeRight, StrafeLeft;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.000, 85.000, Point.CARTESIAN),
                                new Point(15.252, 128.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(22.430, 107.439, Point.CARTESIAN),
                                new Point(45.750, 109.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(45.750, 109.250, Point.CARTESIAN),
                                new Point(15.252, 128.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(22.430, 115.514, Point.CARTESIAN),
                                new Point(45.750, 120.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(45.750, 120.250, Point.CARTESIAN),
                                new Point(15.252, 128.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(45.750, 130.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(45.750, 130.250, Point.CARTESIAN),
                                new Point(15.252, 128.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(55.785, 121.484, Point.CARTESIAN),
                                new Point(65.363, 95.608, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        Middle = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(57.466, 129.046, Point.CARTESIAN),
                                new Point(59.986, 95.608, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                .build();

        StrafeRight = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(59.986, 95.608, Point.CARTESIAN),
                                new Point(84.500, 95.608, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        StrafeLeft = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(84.500, 95.608, Point.CARTESIAN),
                                new Point(60.000, 95.608, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
                Claw.INSTANCE.close(),
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Arm.INSTANCE.toHigh()
                ),

                new ParallelGroup(
                        Lift.INSTANCE.toHigh(),
                        ServoRotire.INSTANCE.outtake()
                ),

                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(grabPickup1),
                        ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.closed()
                ),

                Claw.INSTANCE.close(),

                new ParallelGroup(
                        new FollowPath(scorePickup1),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh()
                ),

                ServoRotire.INSTANCE.outtake(),
                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(grabPickup2),
                        ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.closed()
                ),

                Claw.INSTANCE.close(),

                new ParallelGroup(
                        new FollowPath(scorePickup2),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh()
                ),

                ServoRotire.INSTANCE.outtake(),
                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(grabPickup3),
                        ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.closed()
                ),

                Claw.INSTANCE.close(),

                new ParallelGroup(
                        new FollowPath(scorePickup3),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh()
                ),

                ServoRotire.INSTANCE.outtake(),
                Claw.INSTANCE.open(),

                new ParallelGroup(
                        new FollowPath(Middle),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.closed(),
                        ServoRotire.INSTANCE.intake()
                )
        );
    }

    public SequentialGroup SensorRoutine() {
        return new SequentialGroup(
                Lift.INSTANCE.closed(),

                new FollowPath(StrafeRight),
                new FollowPath(StrafeLeft)
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 85, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
        /// while (color.notEquals("YELLOW")) {
            SensorRoutine().invoke();
        /// }
    }
}