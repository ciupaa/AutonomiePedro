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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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


@Config
@Autonomous(name = "AutoSample")


public class AutoSample extends PedroOpMode {
    public AutoSample() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

   // private PathChain grabPickup1, grabPickup2, grabPickup3,
            //scorePickup1, scorePickup2, scorePickup3,
            //scorePreload, Park;
    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park;

    public void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(9.000, 85.000, Point.CARTESIAN),
                                new Point(34.766, 97.570, Point.CARTESIAN),
                                new Point(18.168, 124.486, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(18.168, 124.486, Point.CARTESIAN),
                                new Point(30.729, 121.346, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(30.729, 121.346, Point.CARTESIAN),
                                new Point(18.168, 124.486, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(18.168, 124.486, Point.CARTESIAN),
                                new Point(21.757, 134.804, Point.CARTESIAN),
                                new Point(30.729, 131.215, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(30.729, 131.215, Point.CARTESIAN),
                                new Point(30.953, 120.000, Point.CARTESIAN),
                                new Point(18.168, 124.486, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(18.168, 124.486, Point.CARTESIAN),
                                new Point(41.720, 113.720, Point.CARTESIAN),
                                new Point(45.308, 129.645, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(45.308, 129.645, Point.CARTESIAN),
                                new Point(41.720, 113.495, Point.CARTESIAN),
                                new Point(18.168, 124.486, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(18.168, 124.486, Point.CARTESIAN),
                                new Point(55.850, 124.935, Point.CARTESIAN),
                                new Point(57.421, 95.551, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();
    }

    public SequentialGroup secondRoutine() {
        return new SequentialGroup(
                Claw.INSTANCE.close(),
            //    ServoRotire.INSTANCE.intake(),
                new ParallelGroup(
                        new FollowPath(scorePreload),
                        Arm.INSTANCE.toHigh()
                ),
              //  ServoRotire.INSTANCE.outtake(),
           //     new Delay(0.5),
              //  Claw.INSTANCE.open(),
                Lift.INSTANCE.toHigh(),
                new ParallelGroup(
                        new FollowPath(grabPickup1),
                     //   ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.toLow()
                ),
               // Claw.INSTANCE.close(),
                new ParallelGroup(
                        new FollowPath(scorePickup1),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh()
                ),
              //  ServoRotire.INSTANCE.outtake(),
               // new Delay(0.5),
             //   Claw.INSTANCE.open(),
                new ParallelGroup(
                        new FollowPath(grabPickup2),
                     //   ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.toLow()
                ),
              //  Claw.INSTANCE.close(),
                new ParallelGroup(
                        new FollowPath(scorePickup2),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh()
                ),
             //   ServoRotire.INSTANCE.outtake(),
             //   new Delay(0.5),
              //  Claw.INSTANCE.open(),
                new ParallelGroup(
                        new FollowPath(grabPickup3),
                       // ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.toLow()
                ),
          //      Claw.INSTANCE.close(),
                new ParallelGroup(
                        new FollowPath(scorePickup3),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh()
                ),
              //  ServoRotire.INSTANCE.outtake(),
              //  new Delay(0.5),
               // Claw.INSTANCE.open(),
                new ParallelGroup(
                        new FollowPath(Park),
                        Arm.INSTANCE.toLow(),
                        Lift.INSTANCE.toLow()
                )
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(9, 85, 0));
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}