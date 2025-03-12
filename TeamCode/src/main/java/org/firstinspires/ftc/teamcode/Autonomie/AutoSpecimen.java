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
import org.firstinspires.ftc.teamcode.SubSystems.ServoRotire;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoSpecimen")
//

public class AutoSpecimen extends PedroOpMode {
    public AutoSpecimen() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }
    /*
    private final Pose startPose = new Pose(9.0, 85.0, Math.toRadians(0.0));
    private final Pose scorePose = new Pose(37.0, 50.0, Math.toRadians(180.0));
    private final Pose pickup1Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(0, 0, Math.toRadians(0));
    private final Pose parkPose = new Pose(0, 0, Math.toRadians(0));
*/
    private PathChain gotoPickup1, gotoPickup2, gotoPickup3, gotoIntermediar,
            pushPickup1, pushPickup2, pushPickup3,
            grabPickup1, grabPickup2, grabPickup3, grabHumanPlayer,
            scorePickup1, scorePickup2, scorePickup3, scoreHumanPlayer,
            scorePreload, Park;

    public void buildPaths() {
        // From AutoSpecimen2, Line 1
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(9.918, 63.551, Point.CARTESIAN),
                                new Point(9.918, 63.551, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // No direct equivalent in AutoSpecimen2, so we'll assume an intermediate step is implied
        gotoIntermediar = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(9.918, 63.551, Point.CARTESIAN), // After scorePreload
                                new Point(35.500, 63.551, Point.CARTESIAN) // Before pushPickup1
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 2 (adjusted for gotoPickup1)
        gotoPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(35.500, 63.551, Point.CARTESIAN),
                                new Point(35.500, 63.551, Point.CARTESIAN) // Prep for push
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 2
        pushPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(9.918, 63.551, Point.CARTESIAN),
                                new Point(35.500, 63.551, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 3 (adjusted for gotoPickup2)
        gotoPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(35.500, 63.551, Point.CARTESIAN),
                                new Point(18.184, 49.224, Point.CARTESIAN),
                                new Point(34.460, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 4
        pushPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(34.460, 34.000, Point.CARTESIAN),
                                new Point(59.487, 34.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 5 (adjusted for gotoPickup3)
        gotoPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(59.487, 34.000, Point.CARTESIAN),
                                new Point(66.995, 30.995, Point.CARTESIAN),
                                new Point(59.487, 23.487, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 6
        pushPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(59.487, 23.487, Point.CARTESIAN),
                                new Point(17.326, 20.406, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 7 (adjusted for grabPickup1)
        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(17.326, 20.406, Point.CARTESIAN),
                                new Point(61.604, 21.561, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 8 (adjusted for scorePickup1)
        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(61.604, 21.561, Point.CARTESIAN),
                                new Point(71.816, 19.102, Point.CARTESIAN),
                                new Point(60.245, 13.224, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 9 (adjusted for grabPickup2)
        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(60.245, 13.224, Point.CARTESIAN),
                                new Point(16.364, 11.743, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 10 (adjusted for scorePickup2)
        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(16.364, 11.743, Point.CARTESIAN),
                                new Point(59.487, 12.706, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 11 (adjusted for grabPickup3)
        grabPickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(59.487, 12.706, Point.CARTESIAN),
                                new Point(58.909, 4.043, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 12 (adjusted for scorePickup3)
        scorePickup3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(58.909, 4.043, Point.CARTESIAN),
                                new Point(15.016, 3.850, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Line 13 (adjusted for grabHumanPlayer)
        grabHumanPlayer = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(15.016, 3.850, Point.CARTESIAN),
                                new Point(34.898, 21.122, Point.CARTESIAN),
                                new Point(15.429, 29.204, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .build();

        // From AutoSpecimen2, Line 14
        scoreHumanPlayer = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(15.429, 29.204, Point.CARTESIAN),
                                new Point(10.286, 54.184, Point.CARTESIAN),
                                new Point(35.500, 65.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();

        // From AutoSpecimen2, Lines 21-22 combined
        Park = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(35.500, 68.500, Point.CARTESIAN),
                                new Point(15.429, 29.204, Point.CARTESIAN),
                                new Point(35.500, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
        new FollowPath(scorePreload),
                new FollowPath(gotoIntermediar),
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
                new FollowPath(Park)
                );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(9.918, 63.551, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}