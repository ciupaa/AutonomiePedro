package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.dashboard.config.Config;
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
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
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


public class AutoSampleCameraTest extends PedroOpMode {
    private Limelight3A limelight;
    public AutoSampleCameraTest() {
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
                Claw.INSTANCE.open()
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 85, 0));
        buildPaths();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
        detectAndApproachSample();
    }

    public void detectAndApproachSample() {
        LLResult result = limelight.getLatestResult(); // Get the latest result from Limelight

        if (result != null && result.isValid()) { // Check if result is valid
            // Accessing the target details from the result
            double targetX = result.getTx(); // Get target X position (horizontal offset)
            double targetY = result.getTy(); // Get target Y position (vertical offset)

            telemetry.addData("Limelight Target", "X: %.2f, Y: %.2f", targetX, targetY);

            // If the number of targets is above a threshold, proceed to move towards it
            moveToSample(targetX, targetY);
        } else {
            telemetry.addData("Limelight", "No valid data from Limelight.");
        }

        telemetry.update();  // Display telemetry data
    }


    public void moveToSample(double targetX, double targetY) {
        // Assuming that targetX and targetY are the offset values of the target
        double moveDistance = 10; // Example: Move 10 inches away from the target

        // Convert offset (in degrees) to movement distance
        double forwardSpeed = (targetY > 0) ? 0.5 : -0.5;  // Forward or backward depending on Y
        double turnSpeed = targetX > 0 ? 0.3 : -0.3; // Turn towards the target X

        // Use your robot's movement logic to move it closer
        driveRobot(forwardSpeed, turnSpeed);
    }

    // Simple movement function
    public void driveRobot(double forwardSpeed, double turnSpeed) {
        // Adjust motor powers for robot drive system
        // For example:
        // leftDrive.setPower(forwardSpeed + turnSpeed);
        // rightDrive.setPower(forwardSpeed - turnSpeed);
    }

    // Function to keep tracking the target if not within range
    public void keepTrackingTarget(double targetX, double targetY) {
        double turnSpeed = targetX > 0 ? 0.3 : -0.3;
        driveRobot(0, turnSpeed); // Adjust robot to face the target
    }

    @Override
    public void onStop() {
        limelight.stop();
    }
}