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
@TeleOp(name = "TeleOpMeetXEOWithIntake")
public class TeleOpTEST extends OpMode {

    // --- Hardware Components ---
    private DcMotorEx motor_stanga;      // Arm motor
    private DcMotorEx motor_glisiere;    // Lift (slides) motor
    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_stanga;
    private DcMotorEx spate_dreapta;
    private DcMotorEx hang1;
    private DcMotorEx hang2;
    private Servo servoRotire;
    private Servo cleste;
    private CRServo hang31;
    private CRServo hang32;

    // --- PID Controllers ---
    private PIDController armController;   // For motor_stanga (from PID)
    private PIDController liftController;  // For motor_glisiere (from TeleOpMeetXEO)
    private PIDController hang1pid;
    private PIDController hang2pid;

    // --- PID Tuning Parameters ---
    // Arm (from PID)
    public static double armP = 0.0055, armI = 0, armD = 0.0002, armF = 0.0011;
    // Lift (from TeleOpMeetXEO)
    public static double liftP = 0.01, liftI = 0, liftD = 0.0002, liftF = 0.14;
    // Hang motors (from PID)
    public static double h1p = 0, h1i = 0, h1d = 0, h1f = 0;
    public static double h2p = 0, h2i = 0, h2d = 0, h2f = 0;

    // --- Target Positions ---
    public static double armTarget = 100;  // New intake base position
    public static double ltarget = 10;     // From liftClosed
    public static double h3target = 0;

    // --- Constants ---
    private static final double TICKS_IN_DEGREE = 2.77;  // From PID
    private static final double TICKS_IN_MM = 3.20;      // From TeleOpMeetXEO
    private static final double HANG_TICKS_IN_MM = 3.434;// From PID
    private static final double FUDGE_FACTOR = 250;      // From PID
    private static final double TIME_WINDOW = 0.5;       // From PID

    // --- Position Presets (from TeleOpMeetXEO) ---
    private static final double ARM_COLLAPSED_INTO_ROBOT = 0;
    private static final double ARM_COS_SUS_BRAT = 115 * (28 * 71.2 * 9.6 / 360.0); // ~3097.92 ticks
    private static final double ARM_COS_JOS_BRAT = 90 * (28 * 71.2 * 9.6 / 360.0);  // ~2426.88 ticks
    private static final double ARM_HANG = 140 * (28 * 71.2 * 9.6 / 360.0);         // ~3776.48 ticks
    private static final double ARM_INTAKE_OLD = 25 * (28 * 71.2 * 9.6 / 360.0);    // ~692.54 ticks (old intake)
    private static final double ARM_HANG_POS1 = 7140;
    private static final double ARM_HANG_POS2 = 8701;
    private static final double LIFT_CLOSED = 10;
    private static final double LIFT_MAX = 1000;
    private static final double LIFT_COS_SUS = 1600;
    private static final double LIFT_COS_JOS = 500;

    // --- Servo Positions ---
    private static final double CLESTE_DESCHIS = 0.6;
    private static final double CLESTE_INCHIS = 0.93;
    private static final double SERVO_TRAS = 0.6;
    private static final double SERVO_RETRAS = 0.4;

    // --- Intake Auto-Adjust Parameters (Placeholders) ---
    private static final double INTAKE_LIFT_THRESHOLD = 10;   // Start adjusting above liftClosed
    private static final double ARM_LIFT_RATIO = 0.06996;     // ~25 degrees over 990 ticks (1000 - 10)
    private static final double INTAKE_BASE_ARM = 100;

    // --- State Variables ---
    private double armPositionFudgeFactor = 0;
    private double cycleTime = 0;
    private double loopTime = 0;
    private double oldTime = 0;
    private boolean inIntakeMode = false;
    private boolean lastAState, lastBState, lastDpadRightState;
    private int aPressCount, bPressCount, dpadRightPressCount;
    private double lastATime, lastBTime, lastDpadRightTime;

    @Override
    public void init() {
        // Initialize PID Controllers
        armController = new PIDController(armP, armI, armD);    // From PID
        liftController = new PIDController(liftP, liftI, liftD); // From TeleOpMeetXEO
        hang1pid = new PIDController(h1p, h1i, h1d);
        hang2pid = new PIDController(h2p, h2i, h2d);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        motor_stanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        fata_dreapta = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");
        cleste = hardwareMap.get(Servo.class, "cleste");
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");
        hang31 = hardwareMap.get(CRServo.class, "hang31");
        hang32 = hardwareMap.get(CRServo.class, "hang32");

        // Configure motor directions
        motor_stanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        fata_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spate_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Bulk caching
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        // Update PID parameters
        armController.setPID(armP, armI, armD);
        liftController.setPID(liftP, liftI, liftD);
        hang1pid.setPID(h1p, h1i, h1d);
        hang2pid.setPID(h2p, h2i, h2d);

        // Get current positions
        int armPos = motor_stanga.getCurrentPosition();
        int liftPos = motor_glisiere.getCurrentPosition();
        int hang1Pos = hang1.getCurrentPosition();

        // Auto-adjust arm in intake mode
        if (inIntakeMode) {
            if (ltarget > INTAKE_LIFT_THRESHOLD) {
                double liftExtension = ltarget - INTAKE_LIFT_THRESHOLD;
                double armAdjustment = liftExtension * ARM_LIFT_RATIO;
                armTarget = INTAKE_BASE_ARM + armAdjustment;
            } else {
                armTarget = INTAKE_BASE_ARM; // Return to base when retracted
            }
        }

        // Calculate PID and feedforward
        double armPid = armController.calculate(armPos, armTarget + (inIntakeMode ? 0 : armPositionFudgeFactor));
        double armFF = cos(toRadians((armTarget + (inIntakeMode ? 0 : armPositionFudgeFactor)) / TICKS_IN_DEGREE)) * armF;
        double armPower = armPid + armFF;

        double liftPid = liftController.calculate(liftPos, ltarget);
        double liftFF = liftF;
        double liftPower = liftPid + liftFF;

        double hangPid = hang1pid.calculate(hang1Pos, h3target);
        double hangFF = h1f;
        double hangPower = hangPid + hangFF;

        // Apply power
        motor_stanga.setPower(armPower);
        motor_glisiere.setPower(liftPower);
        hang1.setPower(hangPower);
        hang2.setPower(hangPower);

        // Drivetrain control (from TeleOpMeetXEO)
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fata_stanga.setPower(frontLeftPower);
        spate_stanga.setPower(backLeftPower);
        fata_dreapta.setPower(frontRightPower);
        spate_dreapta.setPower(backRightPower);

        // Hang servos control (from PID)
        double hangServoPower = gamepad1.right_trigger - gamepad1.left_trigger;
        hang31.setPower(hangServoPower);
        hang32.setPower(hangServoPower);

        // Fudge factor (unused in intake mode)
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger - gamepad2.left_trigger);

        // Lift manual control (from TeleOpMeetXEO)
        if (gamepad2.right_bumper) ltarget += 20;
        if (gamepad2.left_bumper) ltarget -= 20;

        // Arm and lift presets (from TeleOpMeetXEO)
        if (gamepad2.dpad_left) {
            armTarget = INTAKE_BASE_ARM;
            ltarget = LIFT_CLOSED;
            inIntakeMode = true;
        }
        if (gamepad2.dpad_right) {
            armTarget = ARM_COLLAPSED_INTO_ROBOT;
            ltarget = LIFT_CLOSED;
            inIntakeMode = false;
        }
        if (gamepad2.dpad_up) {
            armTarget = ARM_COS_SUS_BRAT;
            if (armPos > 4000) ltarget = LIFT_COS_SUS;
            inIntakeMode = false;
        }
        if (gamepad2.dpad_down) {
            armTarget = ARM_COS_JOS_BRAT;
            ltarget = LIFT_CLOSED;
            inIntakeMode = false;
        }
        if (gamepad1.dpad_right) {
            armTarget = ARM_HANG_POS1;
            inIntakeMode = false;
        }
        if (gamepad1.dpad_right && armPos > 8600) {
            armTarget = ARM_HANG_POS2;
            inIntakeMode = false;
        }
        if (gamepad1.dpad_left) {
            armTarget = ARM_HANG;
            inIntakeMode = false;
        }
        if (gamepad1.a) {
            armTarget = ARM_COLLAPSED_INTO_ROBOT;
            inIntakeMode = false;
        }

        // Servo controls (from TeleOpMeetXEO)
        if (gamepad2.a) cleste.setPosition(CLESTE_INCHIS);
        if (gamepad2.x) cleste.setPosition(CLESTE_DESCHIS);
        if (gamepad2.b) servoRotire.setPosition(SERVO_RETRAS);
        if (gamepad2.y) servoRotire.setPosition(SERVO_TRAS);

        // Enforce limits (from TeleOpMeetXEO)
        if (liftPos > LIFT_COS_SUS) ltarget -= abs(LIFT_COS_SUS - ltarget);
        if (liftPos < 20) ltarget += abs(0 - ltarget); // Adjusted to match TeleOpMeetXEO
        if (armPos < ARM_COLLAPSED_INTO_ROBOT) armTarget = ARM_COLLAPSED_INTO_ROBOT;
        if (armPos < 45 * (28 * 71.2 * 9.6 / 360.0) && ltarget > 1200) ltarget = 1200;

        // Update timing
        loopTime = getRuntime();
        cycleTime = loopTime - oldTime;
        oldTime = loopTime;

        // Telemetry
        telemetry.addLine("PETUNIX");
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Lift Position", liftPos);
        telemetry.addData("Hang Position", hang1Pos);
        telemetry.addData("Arm Target", armTarget);
        telemetry.addData("Lift Target", ltarget);
        telemetry.addData("In Intake Mode", inIntakeMode);
        telemetry.addLine("ROBOPEDA");
        telemetry.addLine("CIUPA, LUCAS & GEORGE");
        telemetry.update();
    }
}