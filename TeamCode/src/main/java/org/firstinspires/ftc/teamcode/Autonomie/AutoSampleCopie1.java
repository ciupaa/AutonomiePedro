package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
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
@Autonomous(name = "AutoSampleCopie1")
public class AutoSampleCopie1 extends PedroOpMode {
    public AutoSampleCopie1() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.000, 85.000, Point.CARTESIAN),
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN),
                                new Point(22.430, 107.439, Point.CARTESIAN),
                                new Point(40, 109.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(40, 109.250, Point.CARTESIAN),
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN),
                                new Point(22.430, 115.514, Point.CARTESIAN),
                                new Point(40, 120.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(40, 120.250, Point.CARTESIAN),
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN),
                                new Point(40, 130.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(40, 130.250, Point.CARTESIAN),
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(12.306122448979592, 132.04081632653063, Point.CARTESIAN),
                                new Point(55.785, 121.484, Point.CARTESIAN),
                                new Point(65.363, 95.608, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
                // Score Preload
                Claw.INSTANCE.close(),
                new Delay(0.5),
                new ParallelGroup(
                        Arm.INSTANCE.toHigh(),
                        new FollowPath(scorePreload),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(0.5),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                new Delay(0.4),
                Arm.INSTANCE.toHighUp(),
                new Delay(0.7),

                // Grab Pickup 1
                new ParallelGroup(
                        Lift.INSTANCE.closed(),
                        new FollowPath(grabPickup1),
                        new Delay(1),
                        Arm.INSTANCE.grabIntake(),
                        ServoRotire.INSTANCE.intake()
                ),
                new Delay(1),
                Lift.INSTANCE.intake(),
                new Delay(1),
                ServoRotire.INSTANCE.outtake(),
        new Delay(1),
                Claw.INSTANCE.close(),
                ServoRotire.INSTANCE.outtake(),


                // Score Pickup 1
                new ParallelGroup(
                        new FollowPath(scorePickup1),
                        new Delay(0.5),
                        Arm.INSTANCE.toHigh(),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(1),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                new Delay(0.5),
                Arm.INSTANCE.toHighUp(),
                ServoRotire.INSTANCE.outtake(),
                new Delay(0.8),
                Lift.INSTANCE.closed(),

                // Grab Pickup 2
                new ParallelGroup(
                        new FollowPath(grabPickup2),
                        Arm.INSTANCE.grabIntake(),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(1),
                Lift.INSTANCE.intake(),
                new Delay(0.5),
                Claw.INSTANCE.close(),
                new Delay(1),

                // Score Pickup 2
                new ParallelGroup(
                        Arm.INSTANCE.toHigh(),
                        new FollowPath(scorePickup2),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(1),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                new Delay(0.5),
                Arm.INSTANCE.toHighUp(),
                ServoRotire.INSTANCE.intake(),
                new Delay(0.5),
                Lift.INSTANCE.closed(),

                // Grab Pickup 3
                new ParallelGroup(
                        new FollowPath(grabPickup3),
                        Arm.INSTANCE.grabIntake(),
                        ServoRotire.INSTANCE.intake()
                ),
                new Delay(1),
                Lift.INSTANCE.intake(),
                new Delay(0.5),
                Claw.INSTANCE.close(),
                new Delay(1),
                // Score Pickup 3
                new ParallelGroup(
                        Arm.INSTANCE.toHigh(),
                        new FollowPath(scorePickup3),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(1),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                new Delay(0.5),
                Arm.INSTANCE.toHighUp(),
                ServoRotire.INSTANCE.intake(),
                new Delay(0.5),
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
        follower.setStartingPose(new Pose(7, 85, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}