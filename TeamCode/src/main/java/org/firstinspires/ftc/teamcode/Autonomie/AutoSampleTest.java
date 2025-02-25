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
@Autonomous(name = "AutoSample")


public class AutoSampleTest extends PedroOpMode {
    public AutoSampleTest() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park;

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
                        new BezierLine(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(34.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(34.000, 121.000, Point.CARTESIAN),
                                new Point(15.252, 128.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(34.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(34.000, 131.000, Point.CARTESIAN),
                                new Point(15.252, 128.523, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
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
                        new BezierLine(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(61.458, 108.336, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
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
                        new FollowPath(Park),
                        Arm.INSTANCE.closed(),
                        Lift.INSTANCE.closed(),
                        ServoRotire.INSTANCE.intake()
                )
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
    }
}