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
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.SubSystems.ServoRotire;
import org.firstinspires.ftc.teamcode.SubSystems.Spec;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoSpecimen4")
public class AutoSpecimen4 extends PedroOpMode {
    public static int upPos = 2000;
    public static int Closed = 20;
    public static int Outtt = 1000;
    public AutoSpecimen4() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE, Spec.INSTANCE);
    }

    private PathChain scorePreload, gotoIntermediar,  gotoIntermediar2,gotoPickup1, pushPickup1,
            gotoPickup2, pushPickup2, gotoPickup3, pushPickup3, grabPickup1,
            finalPosition, path12, path13, path14, path15, getGrabPickup2, score2, score21,score3, score31, getGetGrabPickup3;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(10.016, 61.000, Point.CARTESIAN),
                                new Point(37, 57, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        gotoIntermediar = follower.pathBuilder()

                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(37.000, 57.000, Point.CARTESIAN),
                                new Point(24.7, 35.5, Point.CARTESIAN),
                                new Point(50, 34.1, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        gotoIntermediar2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(48.2, 34.1, Point.CARTESIAN),
                                new Point(60, 32.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        gotoPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(60, 33.052, Point.CARTESIAN),
                                new Point(58, 21.495, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(58, 21.495, Point.CARTESIAN),
                                new Point(17, 20.724, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gotoPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                    new Point(14.000, 20.724, Point.CARTESIAN),
                                new Point(67.994, 24.191, Point.CARTESIAN),
                                new Point(55, 12.249, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(55, 12.249, Point.CARTESIAN),
                                new Point(17, 8, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(17, 8, Point.CARTESIAN),
                                new Point(26.389, 8.204, Point.CARTESIAN),
                                new Point(19.454, 35, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        finalPosition = follower.pathBuilder()
                .addPath(

                        // Line 10
                        new BezierLine(
                                new Point(19.454, 35, Point.CARTESIAN),
                                new Point(8.8, 35, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path12 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        /*
                        new BezierCurve(
                                new Point(8.8, 35, Point.CARTESIAN),
                                new Point(39.5, 60, Point.CARTESIAN)
                        )
                                                */
                        new BezierCurve(
                                new Point(8.800, 35.000, Point.CARTESIAN),
                                new Point(16.556, 60.642, Point.CARTESIAN),
                                new Point(39.500, 60.000, Point.CARTESIAN)
                        )
                )

                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        getGrabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(39.5, 60, Point.CARTESIAN),
                                new Point(26.389, 8.204, Point.CARTESIAN),
                                new Point(19.454, 35, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        score2 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(19.454, 35, Point.CARTESIAN),
                                new Point(8.8, 35, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score21 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(8.8, 35, Point.CARTESIAN),
                                new Point(39, 63, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();
        getGetGrabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(39, 63, Point.CARTESIAN),
                                new Point(26.389, 8.204, Point.CARTESIAN),
                                new Point(19.454, 35, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        score3 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(19.454, 35, Point.CARTESIAN),
                                new Point(8.5, 35, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        score31 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(8.5, 35, Point.CARTESIAN),
                                new Point(38, 70, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();



    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
                Arm.INSTANCE.intake(),
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Spec.INSTANCE.Up()
                ),
                new Delay(0.1),
                Spec.INSTANCE.outTake(),
                new Delay(0.1),
                new ParallelGroup(
                        new FollowPath(gotoIntermediar),
                        Spec.INSTANCE.closed()
                        ),
                new FollowPath(gotoIntermediar2),
                new FollowPath(gotoPickup1),
                new FollowPath(pushPickup1),
                new FollowPath(gotoPickup2),
                new FollowPath(pushPickup2),
                new FollowPath(grabPickup1),
                new FollowPath(finalPosition),
                new Delay(0.1),
                new ParallelGroup(
                        new FollowPath(path12),
                        Spec.INSTANCE.Up()
                        ),
                new Delay(0.1),
                Spec.INSTANCE.outTake(),
                new ParallelGroup(
                        new FollowPath(getGrabPickup2),
                        Spec.INSTANCE.closed()
                        ),
                new FollowPath(score2),
                new Delay(0.1),
                new ParallelGroup(
                        new FollowPath(score21),
                        Spec.INSTANCE.Up()
                ),
                new Delay(0.1),
                Spec.INSTANCE.outTake(),
                new ParallelGroup(
                        new FollowPath(getGetGrabPickup3),
                        Spec.INSTANCE.closed()
                        ),
                new FollowPath(score3),
                Spec.INSTANCE.closed(),
                new Delay(0.1),
                new ParallelGroup(
                        new FollowPath(score31),
                        Spec.INSTANCE.Up()
                ),
                new Delay(0.1),
                Spec.INSTANCE.outTake()
                );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(10.016, 61.000, Math.toRadians(180)));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}