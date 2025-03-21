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
import com.qualcomm.robotcore.hardware.Servo;
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
@Autonomous(name = "AutoSampleCopie4")
public class AutoSampleCopie4 extends PedroOpMode {
    public AutoSampleCopie4() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }
    public static int lift1 = 250;
    public static int lift2 = 100;

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(6.353, 104.727, Point.CARTESIAN),
                                new Point(17.134, 123.209, Point.CARTESIAN),
                                new Point(10.203, 133.604, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(14, 128.5, Point.CARTESIAN),
                                new Point(15.209, 96.449, Point.CARTESIAN),
                                new Point(44.5, 108.193, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();


        scorePickup1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(44.5, 108.193, Point.CARTESIAN),
                                new Point(14, 128.5, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(14, 128.5, Point.CARTESIAN),
                                new Point(24.834, 103.957, Point.CARTESIAN),
                                new Point(45.241, 117.818, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(45.241, 117.818, Point.CARTESIAN),
                                new Point(14, 128.5, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(14, 128.5, Point.CARTESIAN),
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
                new ParallelGroup(
                        Arm.INSTANCE.toHigh(),
                        Claw.INSTANCE.close()
                        ),
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        ServoRotire.INSTANCE.autoOut()
                ),
                new Delay(0.4),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open()
                /*
                new Delay(0.1),
                new ParallelGroup(
                        ServoRotire.INSTANCE.intake(),
                        Lift.INSTANCE.closed(),
                        new FollowPath(grabPickup1)
                ),
                Arm.INSTANCE.closed(),
                Lift.INSTANCE.l1(),
                new Delay(0.1),
                Claw.INSTANCE.close()

                 */
                /*
                new ParallelGroup(
                        Claw.INSTANCE.close(),
                        ServoRotire.INSTANCE.outtake(),
                        Arm.INSTANCE.toHigh(),
                        new FollowPath(scorePreload)
                ),
                new Delay(0.2),
                Lift.INSTANCE.toHigh(),
                new Delay(0.3),
                Claw.INSTANCE.open(),
                new Delay(0.5),
                // Grab Pickup 1
                new ParallelGroup(
                        Lift.INSTANCE.l1(),
                        new FollowPath(grabPickup1),
                        ServoRotire.INSTANCE.outtake()
                        ),
                new Delay(0.4),
                Arm.INSTANCE.paralel(),
                new Delay(0.3),
                Arm.INSTANCE.grabIntake(),
                new Delay(0.3),
                Claw.INSTANCE.close(),
                new Delay(0.3),
                // Score Pickup 1
                new ParallelGroup(
                        Arm.INSTANCE.toHigh(),
                        new FollowPath(scorePickup1),
                        Lift.INSTANCE.closed(),
                        ServoRotire.INSTANCE.outtake()
                        ),
                new Delay(0.2),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                // Grab Pickup 2
                new Delay(0.1),
                new ParallelGroup(
                        Lift.INSTANCE.l2(),
                        new FollowPath(grabPickup2),
                        ServoRotire.INSTANCE.outtake()

                ),
                new Delay(0.5),
                Arm.INSTANCE.paralel(),
                new Delay(0.4),
                Arm.INSTANCE.grabIntake(),
                new Delay(0.3),
                Claw.INSTANCE.close(),
                new Delay(0.3),
                // Score Pickup 2
                new ParallelGroup(
                        Arm.INSTANCE.toHigh(),
                        new FollowPath(scorePickup2),
                        Lift.INSTANCE.closed(),
                        ServoRotire.INSTANCE.outtake()
                ),
                new Delay(0.2),
                Lift.INSTANCE.toHigh(),
                new Delay(0.5),
                Claw.INSTANCE.open(),
                // Grab Pickup 2
                new Delay(0.2),
                Lift.INSTANCE.closed()
        );

                 */
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(6.353, 104.727, Math.toRadians(90)));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}