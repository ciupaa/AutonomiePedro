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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoSampleCopie3")
public class AutoSampleCopie3 extends PedroOpMode {
    public AutoSampleCopie3() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1 & 2 combined
                        new BezierLine(
                                new Point(8.082, 110.204, Point.CARTESIAN),
                                new Point(16.941176470588236, 127.8288770053476, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(-47))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(16.941, 127.829, Point.CARTESIAN),
                                new Point(20.214, 99.529, Point.CARTESIAN),
                                new Point(38.5, 107, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(65))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(26.449, 125.082, Point.CARTESIAN),
                                new Point(16.941176470588236, 127.8288770053476, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-12), Math.toRadians(-47))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(16.941, 127.829, Point.CARTESIAN),
                                new Point(16.556, 114.545, Point.CARTESIAN),
                               // new Point(26.449, 125.082, Point.CARTESIAN)
                                 new Point(37.15508021390374, 119.55080213903743, Point.CARTESIAN)

                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(60))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(26.449, 125.082, Point.CARTESIAN),
                                new Point(16.941176470588236, 127.8288770053476, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(19), Math.toRadians(-47))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(16.941176470588236, 127.8288770053476, Point.CARTESIAN),
                                new Point(29.020, 130.224, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(39))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(29.020, 130.224, Point.CARTESIAN),
                                new Point(16.941176470588236, 127.8288770053476, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(39), Math.toRadians(-47))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(16.941176470588236, 127.8288770053476, Point.CARTESIAN),
                                new Point(62.449, 122.510, Point.CARTESIAN),
                                new Point(70.531, 104.878, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-47), Math.toRadians(270))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
                // Score Preload
                Claw.INSTANCE.close(),
                new Delay(0.5),
                new ParallelGroup(
                        Arm.INSTANCE.armBack(),
                        new FollowPath(scorePreload)
                ),
                new Delay(0.2),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                ServoRotire.INSTANCE.autoOut(),
                new Delay(0.3),
                Claw.INSTANCE.autoOpen(),
                new Delay(0.6),
                ServoRotire.INSTANCE.intake(),
                Claw.INSTANCE.open(),
                new Delay(0.4),
                Claw.INSTANCE.close(),
                new Delay(0.4),

                // Grab Pickup 1
                new ParallelGroup(
                        Lift.INSTANCE.toIntakeSpecimen(),
                        new FollowPath(grabPickup1),
                        Claw.INSTANCE.open(),
                        ServoRotire.INSTANCE.outtake()

                ),
                new Delay(0.5),
                Arm.INSTANCE.armPick(),
                new Delay(0.3),
                Arm.INSTANCE.armOut(),
                new Delay(0.4),
                Claw.INSTANCE.close(),
                ServoRotire.INSTANCE.outtake(),
                new Delay(0.3),


                // Score Pickup 1
                new ParallelGroup(
                        Arm.INSTANCE.armBack(),
                        new FollowPath(scorePickup1),
                        Lift.INSTANCE.closed()
                        ),
                new Delay(0.2),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                ServoRotire.INSTANCE.autoOut(),
                new Delay(0.3),
                Claw.INSTANCE.autoOpen(),
                new Delay(0.6),
                ServoRotire.INSTANCE.intake(),
                Claw.INSTANCE.open(),
                new Delay(0.4),
                Claw.INSTANCE.close(),
                new Delay(0.4),
                // Grab Pickup 2
                new ParallelGroup(
                        Lift.INSTANCE.grab2(),
                        new FollowPath(grabPickup2),
                        new Delay(1),
                        Arm.INSTANCE.armPick(),
                        Claw.INSTANCE.open(),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(0.5),
                Claw.INSTANCE.close(),
                new Delay(1),

                // Score Pickup 2
                new ParallelGroup(
                        Arm.INSTANCE.armBack(),
                        new FollowPath(scorePickup2)
                ),
                new Delay(0.2),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                ServoRotire.INSTANCE.autoOut(),
                new Delay(0.3),
                Claw.INSTANCE.autoOpen(),
                new Delay(0.6),
                ServoRotire.INSTANCE.intake(),
                Claw.INSTANCE.open(),
                new Delay(0.4),
                Claw.INSTANCE.open(),
                new Delay(0.4),
                // Grab Pickup 3
                new ParallelGroup(
                        new FollowPath(grabPickup3),
                        new Delay(1),
                        Lift.INSTANCE.closed(),
                        Arm.INSTANCE.grabIntake(),
                        ServoRotire.INSTANCE.intake()
                ),
                new Delay(0.3),
                Lift.INSTANCE.intake(),
                new Delay(0.5),
                Claw.INSTANCE.close(),
                new Delay(1),
                // Score Pickup 3
                new ParallelGroup(
                        Arm.INSTANCE.armBack(),
                        new FollowPath(scorePickup3),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(0.5),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                new Delay(0.3),
                Lift.INSTANCE.closed(),

                // Park
                new ParallelGroup(
                        new FollowPath(Park),
                        Arm.INSTANCE.hangPos1(),
                        Lift.INSTANCE.closed(),
                        ServoRotire.INSTANCE.intake()
                ),
                new Delay(1) // Added for consistency with previous steps
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(8.082, 110.204, Math.toRadians(270)));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}