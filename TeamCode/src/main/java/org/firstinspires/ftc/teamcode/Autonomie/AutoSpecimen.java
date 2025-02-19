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
import org.firstinspires.ftc.teamcode.SubSystems.ServoRotire;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


import java.io.File;

@Config
@Autonomous(name = "AutoSpecimen")


public class AutoSpecimen extends PedroOpMode {
    public AutoSpecimen() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }
    private final Pose startPose = new Pose(9.0, 85.0, Math.toRadians(0.0));
    private final Pose scorePose = new Pose(37.0, 50.0, Math.toRadians(180.0));
    private final Pose pickup1Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(0));

    private PathChain gotoPickup1, gotoPickup2, gotoPickup3,
            pushPickup1, pushPickup2, pushPickup3,
            grabPickup1, grabPickup2, grabPickup3, grabHumanPlayer,
            scorePickup1, scorePickup2, scorePickup3, scoreHumanPlayer,
            scorePreload, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(134.000, 85.000, Point.CARTESIAN),
                                new Point(106.000, 78.056, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        gotoPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(106.000, 78.056, Point.CARTESIAN),
                                new Point(114.617, 124.037, Point.CARTESIAN),
                                new Point(78.953, 97.794, Point.CARTESIAN),
                                new Point(85.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        pushPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(85.000, 121.000, Point.CARTESIAN),
                                new Point(120.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        gotoPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(120.000, 121.000, Point.CARTESIAN),
                                new Point(80.523, 120.449, Point.CARTESIAN),
                                new Point(85.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        pushPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(85.000, 131.000, Point.CARTESIAN),
                                new Point(120.000, 131.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        gotoPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(120.000, 131.000, Point.CARTESIAN),
                                new Point(80.299, 130.318, Point.CARTESIAN),
                                new Point(85.000, 137.400, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        pushPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(85.000, 137.400, Point.CARTESIAN),
                                new Point(120.000, 137.400, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(120.000, 137.400, Point.CARTESIAN),
                                new Point(93.981, 129.645, Point.CARTESIAN),
                                new Point(134.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(134.000, 121.000, Point.CARTESIAN),
                                new Point(137.271, 76.935, Point.CARTESIAN),
                                new Point(106.000, 75.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(106.000, 75.000, Point.CARTESIAN),
                                new Point(115.514, 122.019, Point.CARTESIAN),
                                new Point(134.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(134.000, 121.000, Point.CARTESIAN),
                                new Point(136.374, 74.467, Point.CARTESIAN),
                                new Point(106.000, 72.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(106.000, 72.000, Point.CARTESIAN),
                                new Point(114.168, 121.346, Point.CARTESIAN),
                                new Point(134.000, 121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(134.000, 121.000, Point.CARTESIAN),
                                new Point(135.028, 71.776, Point.CARTESIAN),
                                new Point(106.000, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        grabHumanPlayer = follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(106.000, 69.000, Point.CARTESIAN),
                                new Point(113.944,123.140, Point.CARTESIAN),
                                new Point(134.000,121.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        scoreHumanPlayer = follower.pathBuilder()
                .addPath(
                        // Line 15
                        new BezierCurve(
                                new Point(134.000, 121.000, Point.CARTESIAN),
                                new Point(133.682,69.084, Point.CARTESIAN),
                                new Point(106.000,66.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(106.000, 66.000, Point.CARTESIAN),
                                new Point(133.458, 79.626, Point.CARTESIAN),
                                new Point(133.458, 109.682, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
             new FollowPath(scorePreload),
                new FollowPath(gotoPickup1),
                new FollowPath(pushPickup1),
                new FollowPath(gotoPickup2),
                new FollowPath(pushPickup2),
                new FollowPath(gotoPickup3),
                new FollowPath(pushPickup3),
                new FollowPath(grabPickup1),
                new FollowPath(scorePickup1),
                new FollowPath(grabPickup2),
                new FollowPath(scorePickup2),
                new FollowPath(grabPickup3),
                new FollowPath(scorePickup3),
                new FollowPath(grabHumanPlayer),
                new FollowPath(scoreHumanPlayer),
                new FollowPath(Park)
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(134, 85, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}