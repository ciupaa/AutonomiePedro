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
//

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

    private PathChain gotoPickup1, gotoPickup2, gotoPickup3, gotoIntermediar,
            pushPickup1, pushPickup2, pushPickup3,
            grabPickup1, grabPickup2, grabPickup3, grabHumanPlayer,
            scorePickup1, scorePickup2, scorePickup3, scoreHumanPlayer,
            scorePreload, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(9.000, 60.000, Point.CARTESIAN),
                                new Point(37.000, 65.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gotoIntermediar = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(37.000, 65.000, Point.CARTESIAN),
                                new Point(32.299, 39.701, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gotoPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(32.299, 39.701, Point.CARTESIAN),
                                new Point(66.393, 38.579, Point.CARTESIAN),
                                new Point(60.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(60.000, 23.000, Point.CARTESIAN),
                                new Point(25.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gotoPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(25.000, 23.000, Point.CARTESIAN),
                                new Point(64.374, 26.916, Point.CARTESIAN),
                                new Point(60.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(60.000, 13.000, Point.CARTESIAN),
                                new Point(25.000, 13.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        gotoPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(25.000, 13.000, Point.CARTESIAN),
                                new Point(60.336, 14.579, Point.CARTESIAN),
                                new Point(60.000, 7.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        pushPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(60.000, 7.000, Point.CARTESIAN),
                                new Point(25.000, 7.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(25.000, 7.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(37.000, 67.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(37.000, 67.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(37.000, 69.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(37.000, 69.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(37.000, 71.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        grabHumanPlayer = follower.pathBuilder()
                .addPath(
                        // Line 15
                        new BezierLine(
                                new Point(37.000, 71.000, Point.CARTESIAN),
                                new Point(13.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        scoreHumanPlayer = follower.pathBuilder()
                .addPath(
                        // Line 16
                        new BezierLine(
                                new Point(13.000, 24.000, Point.CARTESIAN),
                                new Point(37.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        // Line 17
                        new BezierLine(
                                new Point(37.000, 73.000, Point.CARTESIAN),
                                new Point(13.000, 32.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(

            Claw.INSTANCE.close(),

            new ParallelGroup(
                 new FollowPath(scorePreload),
                 Arm.INSTANCE.toSpecOutTake()
            ),

            Arm.INSTANCE.toSpecGo(),
            Claw.INSTANCE.open(),

            new FollowPath(gotoIntermediar),
            new FollowPath(gotoPickup1),
            new FollowPath(pushPickup1),
            new FollowPath(gotoPickup2),
            new FollowPath(pushPickup2),
            new FollowPath(gotoPickup3),
            new FollowPath(pushPickup3),

            new ParallelGroup(
                    new FollowPath(grabPickup1),
                    Arm.INSTANCE.toSpecIntake()
            ),

            Claw.INSTANCE.close(),

            new ParallelGroup(
                    new FollowPath(scorePickup1),
                    Arm.INSTANCE.toSpecOutTake()
            ),

            Arm.INSTANCE.toSpecGo(),
            Claw.INSTANCE.open(),

            new ParallelGroup(
                    new FollowPath(grabPickup2),
                    Arm.INSTANCE.toSpecIntake()
            ),

            Claw.INSTANCE.close(),

            new ParallelGroup(
                    new FollowPath(scorePickup2),
                    Arm.INSTANCE.toSpecOutTake()
            ),

            Arm.INSTANCE.toSpecGo(),
            Claw.INSTANCE.open(),

            new ParallelGroup(
                    new FollowPath(grabPickup3),
                    Arm.INSTANCE.toSpecIntake()
            ),

            Claw.INSTANCE.close(),

            new ParallelGroup(
                    new FollowPath(scorePickup3),
                    Arm.INSTANCE.toSpecOutTake()
            ),

            Arm.INSTANCE.toSpecGo(),
            Claw.INSTANCE.open(),

            new ParallelGroup(
                    new FollowPath(grabHumanPlayer),
                    Arm.INSTANCE.toSpecIntake()
            ),

            Claw.INSTANCE.close(),

            new ParallelGroup(
                    new FollowPath(scoreHumanPlayer),
                    Arm.INSTANCE.toSpecOutTake()
            ),

            Arm.INSTANCE.toSpecGo(),
            Claw.INSTANCE.open(),

            new ParallelGroup(
                new FollowPath(Park),
                Arm.INSTANCE.toIntake(),
                Lift.INSTANCE.closed()
            )
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(9, 60, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}