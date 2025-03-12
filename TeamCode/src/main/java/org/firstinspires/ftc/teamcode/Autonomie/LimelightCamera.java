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

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@Config
@Autonomous(name = "LimelightCamera")
public class LimelightCamera extends PedroOpMode {
    public LimelightCamera() {
        super(Claw.INSTANCE, Lift.INSTANCE, ServoRotire.INSTANCE, Arm.INSTANCE);
    }

    private PathChain scorePreload, grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, Park;
    private PathChain dynamicPathToTarget; // Dynamic path based on Limelight data

    // Limelight instance
    private Limelight3A limelight;

    // Constants for Limelight-based path generation
    private static final double DISTANCE_SCALE = 24.0; // Example distance in inches (adjust based on field and setup)

    public void buildPaths() {
        // Static paths from AutoSample, excluding dynamicPathToTarget
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
                        new BezierCurve(
                                new Point(15.252, 128.523, Point.CARTESIAN),
                                new Point(55.785, 121.484, Point.CARTESIAN),
                                new Point(65.363, 95.608, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(90))
                .build();

        // Dynamically generate path based on Limelight tx, ty (keep this as is from LimelightSample)
        dynamicPathToTarget = generateDynamicPathFromLimelight();

    }

    // Method to generate a dynamic path based on Limelight data, using FTC and Limelight examples
    private PathChain generateDynamicPathFromLimelight() {
        if (limelight == null) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(0); // Switch to your neural detector pipeline (e.g., for "Into The Deep" objects)
            limelight.start(); // Start polling for data at 100 Hz, as per FTC Quick Start
        }

        LLResult result = limelight.getLatestResult();
        double tx = 0.0, ty = 0.0;

        if (result != null && result.isValid()) {
            // Get neural detector results, as per FTC Pipeline Setup for neural networks
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            if (!detectorResults.isEmpty()) {
                LLResultTypes.DetectorResult detection = detectorResults.get(0); // Use first detection
                tx = detection.getTargetXDegrees(); // Horizontal offset in degrees
                ty = detection.getTargetYDegrees(); // Vertical offset in degrees
                for (LLResultTypes.DetectorResult detectionResult : detectorResults) {

                }
                telemetry.addData("Limelight Detection", "tx: %.2f°, ty: %.2f°", tx, ty);
            }
        } else {
            telemetry.addData("Limelight", "No valid detection");
            return null; // Or return a default path if no target is detected
        }

        // Convert tx, ty degrees to Cartesian points, using trigonometry from Limelight examples
        double txRad = Math.toRadians(tx);
        double tyRad = Math.toRadians(ty);
        double thetaRad = Math.atan2(tyRad, txRad); // Resultant angle in radians

        // Calculate target point using DISTANCE_SCALE (e.g., 24 inches, as per field calibration)
        double targetX = DISTANCE_SCALE * Math.cos(thetaRad);
        double targetY = DISTANCE_SCALE * Math.sin(thetaRad);

        // Get the robot's current position, as per FTC autonomous navigation examples
        Pose currentPose = follower.getPose();
        double startX = currentPose.getX();
        double startY = currentPose.getY();

        // Create a BezierLine path from current position to target, as per pedropathing examples
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(startX, startY, Point.CARTESIAN),
                                new Point(targetX, targetY, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(currentPose.getHeading(), Math.atan2(targetY, targetX)) // Adjust heading
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

                // Replace grabPickup1 with the dynamic path based on Limelight
                new ParallelGroup(
                        new FollowPath(dynamicPathToTarget),
                        ServoRotire.INSTANCE.intake(),
                        Arm.INSTANCE.intake(),
                        Lift.INSTANCE.closed()
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
                        Arm.INSTANCE.intake(),
                        Lift.INSTANCE.closed()
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
                        Arm.INSTANCE.intake(),
                        Lift.INSTANCE.closed()
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
                        new FollowPath(Park),
                        Arm.INSTANCE.closed(),
                        Lift.INSTANCE.closed(),
                        ServoRotire.INSTANCE.intake()
                )
        );
    }

    @Override
    public void onInit() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7, 85, 0));
        buildPaths();

        // Initialize Limelight, as per FTC Programming Quick Start
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.pipelineSwitch(0); // Use neural detector pipeline for "Into The Deep" objects
            limelight.start(); // Start polling at 100 Hz
            telemetry.addData("Limelight", "Initialized and polling");
        } else {
            telemetry.addData("Limelight", "Not found in hardwareMap");
        }
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        secondRoutine().invoke();
    }
}