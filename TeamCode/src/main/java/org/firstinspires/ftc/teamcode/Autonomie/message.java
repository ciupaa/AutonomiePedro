package org.firstinspires.ftc.teamcode.Autonomie;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public class message {
    /**
     * The tag for logging data through LogCat.
     */
    private final String TAG = "LimelightVision";

    /**
     * Telemetry for logging on the driver station.
     */
    private Telemetry telemetry;

    /**
     * The limelight camera.
     */
    public Limelight3A limelight;

    /**
     * The perspective transformation matrix to convert between camera and robot coordinates.
     */
    private Mat transformMatrix;

    /** The list to store detected samples. */
    //  private List<Sample> samples;

    /**
     * The amount to squish the window vertically by in centimeters.
     */
    public static double WINDOW_SQUISH_OFFSET = 3.0;

    /**
     * The amount to move the window to the right horizontally by in centimeters.
     */
    public static double WINDOW_HORIZONTAL_OFFSET = -1.0;

    /**
     * The amount to move the window forwards vertically by in centimeters.
     */
    public static double WINDOW_VERTICAL_OFFSET = -1.0;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // setup limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.setMsTransmissionInterval(11);

        // define the four source points (corners of the area you want to transform)
        Point[] srcPoints = new Point[4];
        srcPoints[0] = new Point(422, 120);           // top-right
        srcPoints[1] = new Point(125, 110);           // top-left
        srcPoints[2] = new Point(463, 444);          // bottom-right
        srcPoints[3] = new Point(20, 429);          // bottom-left

        // define the four destination points (where you want the points to end up)
        Point[] dstPoints = new Point[4];
        dstPoints[0] = new Point(9.0 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 16.0 * 2.54 + WINDOW_VERTICAL_OFFSET - WINDOW_SQUISH_OFFSET);         // top-right
        dstPoints[1] = new Point(-2.0 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 16.0 * 2.54 + WINDOW_VERTICAL_OFFSET - WINDOW_SQUISH_OFFSET);       // top-left
        dstPoints[2] = new Point(9.0 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 4.0 * 2.54 + WINDOW_VERTICAL_OFFSET);         // bottom-right
        dstPoints[3] = new Point(-2.0 * 2.54 + WINDOW_HORIZONTAL_OFFSET, 4.0 * 2.54 + WINDOW_VERTICAL_OFFSET);       // bottom-left

        // convert points to MatOfPoint2f format
        MatOfPoint2f sourceMat = new MatOfPoint2f();
        sourceMat.fromArray(srcPoints);
        MatOfPoint2f destMat = new MatOfPoint2f();
        destMat.fromArray(dstPoints);

        // calculate the perspective transform matrix
        transformMatrix = Imgproc.getPerspectiveTransform(sourceMat, destMat);

        // set the telemetry
        this.telemetry = telemetry;

        // initialize the samples list
        //  samples = new ArrayList<>();
    }

    /**
     * Expected aspect ratio of a sample (long side/shorter side)
     */
    static final double SAMPLE_ASPECT_RATIO = (double) 7 / 3;

    /**
     * Determines whether the sample is horizontal or vertical
     *
     * @param box the coordinates of opposite corners of the sample's bounding box
     * @return the rotation of the sample
     */
/*
    private Sample.Rotation calcRotation(List<List<Double>> box) {
        // obtain the corner coordinates by extrapolating the other two corners from the given ones
        Point[] corners = new Point[4];
        corners[0] = new Point(box.get(0).get(0), box.get(0).get(1));        // top-left
        corners[1] = new Point(box.get(1).get(0), box.get(1).get(1));        // top-right
        corners[2] = new Point(box.get(2).get(0), box.get(2).get(1));        // bottom-right
        corners[3] = new Point(box.get(3).get(0), box.get(3).get(1));        // bottom-left

        // transform the corner coordinates to robot coordinates
        float[][] robotCorners = new float[4][2];
        for (int i = 0; i < 4; i++) {
            Mat pointMat = new Mat(1, 1, CvType.CV_32FC2);
            pointMat.put(0, 0, new float[]{(float)corners[i].x, (float)corners[i].y});
            Mat resultMat = new Mat();
            Core.perspectiveTransform(pointMat, resultMat, transformMatrix);
            resultMat.get(0, 0, robotCorners[i]);
            pointMat.release();
            resultMat.release();
        }



        // calculate width and height of the sample's bounding box as well as the aspect ratio
        double width = Math.hypot(
                robotCorners[1][0] - robotCorners[0][0],
                robotCorners[1][1] - robotCorners[0][1]
        );
        double height = Math.hypot(
                robotCorners[3][0] - robotCorners[0][0],
                robotCorners[3][1] - robotCorners[0][1]
        );
        double aspectRatio = height / width;
        Log.logInfo(TAG, "Corners: " + Arrays.deepToString(robotCorners));
        Log.logInfo(TAG, "Width: " + width);
        Log.logInfo(TAG, "Height: " + height);
        Log.logInfo(TAG, "Aspect ratio: " + aspectRatio);
        telemetry.addData("Aspect ratio", aspectRatio);

/*
        double q = Math.tan(Math.atan(1 / SAMPLE_ASPECT_RATIO) / 2);
        double q2 = q * q;
        double q3 = q2 * q;
        double q4 = q3 * q;

        double A = aspectRatio;
        double A2 = A * A;

        double numerator = -Math.sqrt(
                        A2 * q4
                        + 2 * A2 * q2
                        + A2
                        + 8 * A * q3
                        - 8 * A * q
                        + q4
                        + 2 * q2
                        + 1 )
                + A * q2
                - A
                + 2 * q;
        double denominator = 2 * A * q
                + q2
                - 1;

        // edge case handling
        if (denominator == 0) {
            numerator = 0;
            denominator = 1;
        }

        double sampleAngle = -2 * Math.atan(numerator / denominator);
        if (aspectRatio > SAMPLE_ASPECT_RATIO) {
            sampleAngle -= Math.PI;
        } else if (aspectRatio < SAMPLE_ASPECT_RATIO) {
            sampleAngle += Math.PI;
        }
        sampleAngle = Math.toDegrees(sampleAngle);
        Log.logInfo(TAG, "Sample angle: " + sampleAngle);


        // determine the rotation based on the aspect ratio
        if (aspectRatio < 1) {
            return Sample.Rotation.HORIZONTAL;
        } else {
            return Sample.Rotation.VERTICAL;
        }

    }

    /** Radius of the intake arm. */
    public static double R = 20.0;

    /**
     * Minimum length of the linear slides.
     */
    public static double MIN_L = 0.0;

    /**
     * Maximum length of the linear slides.
     */
    public static double MAX_L = 36.0;

    /**
     * Maximum length range offset.
     */
    public static double MAX_L_RANGE_OFFSET = 0;

    /**
     * Maximum horizontal range offset.
     */
    public static double MAX_R_RANGE_OFFSET = 0;

    /**
     * Offset of the arm from the start of the slides.
     */
    public static double SLIDE_OFFSET = .5;

    /**
     * Offset of the horizontal pivot from the vertical pivot.
     */
    public static double PIVOTS_OFFSET = 5.5;

    /**
     * Height of the arm pivot above the ground.
     */
    public static double PIVOT_HEIGHT = 12.5;

    /**
     * Offset of the claw from the pivot.
     */
    public static double CLAW_OFFSET = 9;

    /**
     * Maximum angle of the input claw.
     */
    public static double MAX_CLAW_ANGLE = 75.0;

    /**
     * The cutoff for the mid to long range.
     */
    public static double VISION_MID = 35.0;

    /**
     * The weight factor for the x coordinate of the sample when prioritizing samples.
     */
    public static double X_WEIGHT = 0.5;

    /**
     * Update the detected samples
     */
    public void update() {
        // clear samples list and hashmap
        //  samples = new ArrayList<>();

        // get the latest detection results
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detection : detections) {
            // get the corner coordinates of the detected sample
            List<List<Double>> corners = detection.getTargetCorners();
            //  Log.logInfo(TAG, "Box corners: " + corners.toString());
            //Sample.Rotation rotation = calcRotation(corners);
            //Log.logInfo(TAG, "Rotation: " + rotation);
            String className = detection.getClassName();

            // find center
            double x = detection.getTargetXPixels();
            double y = detection.getTargetYPixels();
            telemetry.addData(className, "at (" + x + ", " + y + ")");

            // convert to robot coordinates
            Mat pointMat = new Mat(1, 1, CvType.CV_32FC2);
            pointMat.put(0, 0, new float[]{(float) x, (float) y});
            Mat resultMat = new Mat();
            Core.perspectiveTransform(pointMat, resultMat, transformMatrix);
            float[] robotCoords = new float[2];
            resultMat.get(0, 0, robotCoords);

            // create a sample
            //   Sample sample = new Sample(className, rotation, robotCoords[0], robotCoords[1], SLIDE_OFFSET, R,
            //           CLAW_OFFSET, PIVOT_HEIGHT, PIVOTS_OFFSET);


         /*   // ensure that the sample is within reaching range
            double l = sample.l;
            if (Math.abs(robotCoords[0]) >= R + MAX_R_RANGE_OFFSET || robotCoords[1] <= SLIDE_OFFSET) {
                continue;
            } else if (l <= MIN_L || l >= MAX_L + MAX_L_RANGE_OFFSET) {
                continue;
            } else if (Math.abs(sample.theta) > MAX_CLAW_ANGLE) {
                continue;
            }

            // log the robot coordinates
            telemetry.addData("Robot coordinates", "at (" + robotCoords[0] + ", " + robotCoords[1] + ")");
            telemetry.addData("Theta, Phi, L", sample.theta + ", " + sample.phi + ", " + sample.l);

            // add the sample to the list
            samples.add(sample);
*/
            // Clean up
            pointMat.release();
            resultMat.release();
        }

        // prioritizes samples if they are closer to mid first, then by the right-most sample
    /*    samples.sort((s1, s2) -> {
            if (Math.abs(s1.l - VISION_MID) - s1.x * X_WEIGHT
                    < Math.abs(s2.l - VISION_MID) - s2.x * X_WEIGHT) {
                return -1;
            } else if (Math.abs(s1.l - VISION_MID) - s1.x * X_WEIGHT
                    > Math.abs(s2.l - VISION_MID) - s2.x * X_WEIGHT) {
                return 1;
            } else {
                return Double.compare(s1.x, s2.x);
            }
        });
        Log.logInfo(TAG, samples.toString());
    }

    /** The time in milliseconds to buffer detections when attempting to get nearest sample. */
        // public static long BUFFER_TIME = 500;

        /**
         * Get the closest sample of the given color
         * @param color the color of the sample
         * @return the closest sample of the given color, null if no sample is found
         */
    /*
    public Sample getClosestSample(Sample.Color color) {
        long startTime = System.currentTimeMillis();
        limelight.captureSnapshot("game" + startTime);
        while (Math.abs(System.currentTimeMillis() - startTime) < BUFFER_TIME) {
            for (Sample sample : samples) {
                if (sample.color == color) {
                    return sample;
                } else if (color == Sample.Color.ANY) {
                    return sample;
                }
            }
        }
        return null;
    }

     */
    }
}
