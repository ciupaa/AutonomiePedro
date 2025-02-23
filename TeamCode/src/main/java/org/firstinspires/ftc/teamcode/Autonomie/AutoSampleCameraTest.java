package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Claw;
import org.firstinspires.ftc.teamcode.SubSystems.Lift;
import org.firstinspires.ftc.teamcode.SubSystems.ServoRotire;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "AutoSample")

DcMotorEx fata_stanga;
DcMotorEx fata_dreapta;
DcMotorEx spate_stanga;
DcMotorEx spate_dreapta;
DcMotorEx motor_glisiere;

public class AutoSampleCameraTest extends PedroOpMode {
    private Limelight3A limelight;

    public AutoSampleCameraTest() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park, Mijloc, toScore;

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
                        new BezierLine(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(61.458, 108.336, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        Mijloc = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(62.338, 113.419, Point.CARTESIAN),
                                new Point(64.019, 95.500, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(270))
                .build();
    }

    public Command secondRoutine() {
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
                        Lift.INSTANCE.toLow()
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
                        Lift.INSTANCE.toLow()
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
                        Lift.INSTANCE.toLow()
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
                        new FollowPath(Mijloc),
                        Arm.INSTANCE.toIntake(),
                        Lift.INSTANCE.toLow()
                ),

                new ParallelGroup(
                    Lift.INSTANCE.toMiddle(),
                    ServoRotire.INSTANCE.intake()
                ),

                Camera(),

                Arm.INSTANCE.toLow(),
                Claw.INSTANCE.close(),

                Arm.INSTANCE.toIntake(),
                Lift.INSTANCE.toLow(),

                new ParallelGroup(
                        new FollowPath(toScore),
                        Arm.INSTANCE.toHigh(),
                        Lift.INSTANCE.toHigh(),
                        ServoRotire.INSTANCE.outtake()
                ),

                Claw.INSTANCE.open()
        );
    }

    public void Camera() {
        LLResult result = limelight.getLatestResult();

        double ty, tx, CurrentX, CurrentY;

        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();

            if (tx > 0) {
                fata_dreapta.setPower(-);
                fata_stanga.setPower(+);
                spate_dreapta.setPower(+);
                spate_stanga.setPower(-);
            }
            else {
                fata_dreapta.setPower(+);
                fata_stanga.setPower(-);
                spate_dreapta.setPower(-);
                spate_stanga.setPower(+);
            }

            if (ty > 0) {
                motor_glisiere.setPower(+);
            }
            else {
                motor_glisiere.setPower(-);
            }

            Pose3D botpose = result.getBotpose();

            CurrentX = botpose.getPosition().x;
            CurrentY = botpose.getPosition().y;

            toScore = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(CurrentX, CurrentY, Point.CARTESIAN),
                                    new Point(15.252, 128.523, Point.CARTESIAN)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(135))
                    .build();
        }
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 85, 0));
        buildPaths();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        fata_dreapta = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}