package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp(name = "TESTPidSpecimenSiSample")
public class TESTPidSpecimenSiSample extends OpMode {

    // --- Hardware Components ---
    private DcMotorEx motorStanga;      // Arm motor
    private DcMotorEx motorGlisiere;    // Lift motor
    private DcMotorEx fataStanga;       // Front left drive motor
    private DcMotorEx fataDreapta;      // Front right drive motor
    private DcMotorEx spateStanga;      // Back left drive motor
    private DcMotorEx spateDreapta;     // Back right drive motor
    private DcMotorEx hang1;            // Hang motor 1
    private DcMotorEx hang2;            // Hang motor 2
    private Servo servoRotire;          // Rotation servo
    private Servo cleste;               // Claw servo
    private CRServo hang31;             // Hang servo 1
    private CRServo hang32;             // Hang servo 2

    // --- PID Controllers ---
    private PIDController armController;
    private PIDController liftController;
    private PIDController hang1Controller;
    private PIDController hang2Controller;

    // --- PID Tuning Parameters (Configurable via Dashboard) ---
    public static double armP = 0.0055, armI = 0, armD = 0.0002, armF = 0.0011;
    public static double liftP = 0.01, liftI = 0, liftD = 0.0002, liftF = 0.14;
    public static double hang1P = 0, hang1I = 0, hang1D = 0, hang1F = 0;
    public static double hang2P = 0, hang2I = 0, hang2D = 0, hang2F = 0;

    // --- Target Positions ---
    public static double armTarget = 100;
    public static double liftTarget = 20;
    public static double hangTarget = 0;

    // --- Constants ---
    private static final double TICKS_IN_DEGREE = 2.77;
    private static final double TICKS_IN_MM = 3.20;
    private static final double HANG_TICKS_IN_MM = 3.434;
    private static final double FUDGE_FACTOR = 250;
    private static final double TIME_WINDOW = 0.5;    // Multi-press time window (seconds)
    private static final double ARM_TOLERANCE = 50;   // Arm position tolerance (ticks)

    // --- Position Presets ---
    private static final double ARM_CLOSED = 10;
    private static final double ARM_COS_SUS = 5950;
    private static final double ARM_COS_JOS = 5950;
    private static final double ARM_INTAKE = 1400;
    private static final double ARM_INTAKE_SPECIMEN = 2000;
    private static final double ARM_RUNG = 2100;
    private static final double ARM_OUTTAKE_RUNG = 2000;
    private static final double ARM_HANG_POS1 = 7140;
    private static final double ARM_HANG_POS2 = 8701;
    private static final double ARM_HANG3_DOWN = 10;
    private static final double ARM_HANG3_UP = 100;
    private static final double LIFT_CLOSED = 10;
    private static final double LIFT_MAX = 1000;
    private static final double LIFT_COS_SUS = 1600;
    private static final double LIFT_COS_JOS = 500;
    private static final double CLESTE_DESCHIS = 0.6;
    private static final double CLESTE_INCHIS = 1;
    private static final double SERVO_TRAS = 0.4;
    private static final double SERVO_RETRAS = 0.6;

    // --- State Variables ---
    private double armPositionFudgeFactor = 0;
    private double cycleTime = 0;
    private double loopTime = 0;
    private double oldTime = 0;

    // --- Multi-Press Tracking ---
    private boolean lastYState, lastAState, lastBState, lastDpadRightState;
    private int yPressCount, aPressCount, bPressCount, dpadRightPressCount;
    private double lastYTime, lastATime, lastBTime, lastDpadRightTime;

    // --- Arm Wait Logic ---
    private boolean waitingForArm = false;

    @Override
    public void init() {
        // Initialize PID controllers
        armController = new PIDController(armP, armI, armD);
        liftController = new PIDController(liftP, liftI, liftD);
        hang1Controller = new PIDController(hang1P, hang1I, hang1D);
        hang2Controller = new PIDController(hang2P, hang2I, hang2D);

        // Set up telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        motorStanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        motorGlisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");
        fataStanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        fataDreapta = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        spateStanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");
        spateDreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");
        cleste = hardwareMap.get(Servo.class, "cleste");
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");
        hang31 = hardwareMap.get(CRServo.class, "hang31");
        hang32 = hardwareMap.get(CRServo.class, "hang32");

        // Configure motor directions
        motorStanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motorGlisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        fataStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spateStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior for drive motors
        fataDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fataStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spateDreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spateStanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Enable bulk caching on all Lynx modules
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        // Gamepad setup
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        // Update PID parameters
        armController.setPID(armP, armI, armD);
        liftController.setPID(liftP, liftI, liftD);
        hang1Controller.setPID(hang1P, hang1I, hang1D);
        hang2Controller.setPID(hang2P, hang2I, hang2D);

        // Get current motor positions
        int armPos = motorStanga.getCurrentPosition();
        int liftPos = motorGlisiere.getCurrentPosition();
        int hang1Pos = hang1.getCurrentPosition();

        // Calculate PID and feedforward
        double armPid = armController.calculate(armPos, armTarget + armPositionFudgeFactor);
        double liftPid = liftController.calculate(liftPos, liftTarget);
        double armFF = cos(toRadians((armTarget + armPositionFudgeFactor) / TICKS_IN_DEGREE)) * armF;
        double liftFF = liftF;
        double hangPid = hang1Controller.calculate(hang1Pos, hangTarget);
        double hangFF = hang1F;

        // Apply power to motors
        motorStanga.setPower(armPid + armFF);
        motorGlisiere.setPower(liftPid + liftFF);
        hang1.setPower(hangPid + hangFF);
        hang2.setPower(hangPid + hangFF); // Sync hang2 with hang1

        // Drivetrain control
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double driveRx = gamepad1.right_stick_x;
        double denominator = Math.max(abs(driveY) + abs(driveX) + abs(driveRx), 1);
        fataStanga.setPower((driveY + driveX + driveRx) / denominator);
        spateStanga.setPower((driveY - driveX + driveRx) / denominator);
        fataDreapta.setPower((driveY - driveX - driveRx) / denominator);
        spateDreapta.setPower((driveY + driveX - driveRx) / denominator);

        // Hang servos control
        double hangServoPower = gamepad1.right_trigger - gamepad1.left_trigger;
        hang31.setPower(hangServoPower);
        hang32.setPower(hangServoPower);

        // Update fudge factor
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger - gamepad2.left_trigger);

        // Lift manual control
        if (gamepad2.right_bumper) liftTarget += 20;
        if (gamepad2.left_bumper) liftTarget -= 20;

        // Arm and lift presets
        if (gamepad2.dpad_left) {
            armTarget = ARM_INTAKE;
            liftTarget = LIFT_CLOSED;
        }
        if (gamepad2.dpad_right) {
            armTarget = ARM_CLOSED;
            liftTarget = LIFT_CLOSED;
        }
        if (gamepad2.dpad_up) {
            armTarget = ARM_COS_SUS;
            if (armPos > 4000) liftTarget = LIFT_COS_SUS;
        }
        if (gamepad2.dpad_down) {
            armTarget = ARM_COS_JOS;
            liftTarget = LIFT_CLOSED;
        }

        // Multi-press for gamepad2.y (Specimen sequence)
        updateMultiPress(gamepad2.y, lastYState, yPressCount, lastYTime, () -> {
            switch (yPressCount) {
                case 1: armTarget = ARM_INTAKE_SPECIMEN; break;
                case 2: armTarget = ARM_RUNG; break;
            }
        }, () -> lastYState = gamepad2.y);

        // gamepad2.x with arm wait logic
        if (gamepad2.x && armTarget == ARM_RUNG) {
            armTarget = ARM_OUTTAKE_RUNG;
            waitingForArm = true;
            gamepad1.rumble(1000);
        }
        if (waitingForArm && abs(armPos - ARM_OUTTAKE_RUNG) <= ARM_TOLERANCE) {
            cleste.setPosition(CLESTE_DESCHIS);
            waitingForArm = false;
        }

        // Multi-press for gamepad2.a (Claw toggle)
        updateMultiPress(gamepad2.a, lastAState, aPressCount, lastATime, () -> {
            switch (aPressCount) {
                case 1:
                    cleste.setPosition(CLESTE_INCHIS);
                    gamepad1.rumble(1000);
                    break;
                case 2:
                    cleste.setPosition(CLESTE_DESCHIS);
                    break;
            }
        }, () -> lastAState = gamepad2.a);

        // Multi-press for gamepad2.b (Servo rotation)
        updateMultiPress(gamepad2.b, lastBState, bPressCount, lastBTime, () -> {
            switch (bPressCount) {
                case 1: servoRotire.setPosition(SERVO_TRAS); break;
                case 2: servoRotire.setPosition(SERVO_RETRAS); break;
            }
        }, () -> lastBState = gamepad2.b);

        // Multi-press for gamepad1.dpad_right (Ascent 2)
        updateMultiPress(gamepad1.dpad_right, lastDpadRightState, dpadRightPressCount, lastDpadRightTime, () -> {
            switch (dpadRightPressCount) {
                case 1: armTarget = ARM_HANG_POS1; break;
                case 2: if (armPos > 8600) armTarget = ARM_HANG_POS2; break;
            }
        }, () -> lastDpadRightState = gamepad1.dpad_right);

        // Additional controls
        if (gamepad1.dpad_left && armPos > 8600) armTarget = ARM_CLOSED;
        if (gamepad1.dpad_up) hangTarget = ARM_HANG3_UP;
        if (gamepad1.dpad_down) hangTarget = ARM_HANG3_DOWN;

        // Enforce position limits
        if (liftPos > LIFT_COS_SUS) liftTarget -= abs(LIFT_COS_SUS - liftTarget);
        if (liftPos < 20) liftTarget += abs(0 - liftTarget);
        if (armPos < 0) {
            armTarget = 0;
            armPositionFudgeFactor = 0;
        }

        // Rumble feedback for long loop times
        if (loopTime > 75) {
            gamepad1.rumble(1500);
            gamepad2.rumble(1500);
        }

        // Update timing
        loopTime = getRuntime();
        cycleTime = loopTime - oldTime;
        oldTime = loopTime;

        // Telemetry output
        telemetry.addLine("PETUNIX");
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Lift Position", liftPos);
        telemetry.addData("Hang Position", hang1Pos);
        telemetry.addLine("ROBOPEDA");
        telemetry.addLine("CIUPA, LUCAS & GEORGE");
        telemetry.addLine("PETUNIX");
        telemetry.update();
    }

    // Helper method for multi-press logic
    private void updateMultiPress(boolean currentState, boolean lastState, int pressCountField,
                                  double lastTimeField, Runnable action, Runnable updateState) {
        int pressCount = pressCountField;
        double lastTime = lastTimeField;

        if (currentState && !lastState) {
            double currentTime = getRuntime();
            pressCount = (currentTime - lastTime <= TIME_WINDOW) ? pressCount + 1 : 1;
            lastTime = currentTime;
        }
        if (!currentState && lastState && (getRuntime() - lastTime > TIME_WINDOW)) {
            action.run();
            pressCount = 0;
        }

        // Update fields via reflection-like assignment
        if (currentState != lastState) {
            updateState.run();
            if (pressCountField == yPressCount) yPressCount = pressCount;
            else if (pressCountField == aPressCount) aPressCount = pressCount;
            else if (pressCountField == bPressCount) bPressCount = pressCount;
            else if (pressCountField == dpadRightPressCount) dpadRightPressCount = pressCount;
            if (lastTimeField == lastYTime) lastYTime = lastTime;
            else if (lastTimeField == lastATime) lastATime = lastTime;
            else if (lastTimeField == lastBTime) lastBTime = lastTime;
            else if (lastTimeField == lastDpadRightTime) lastDpadRightTime = lastTime;
        }
    }
}