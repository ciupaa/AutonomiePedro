package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
@TeleOp
public class TeleOpRegionala extends OpMode {

    // Hardware Declarations
    private DcMotorEx motor_stanga;
    private DcMotorEx motor_glisiere;
    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_stanga;
    private DcMotorEx spate_dreapta;
    private Servo servoRotire;
    private Servo cleste;

    // PID Controllers
    private PIDController controller;
    private PIDController lcontroller;

    // PID Tuning Parameters
    public static double p = 0.0055, i = 0, d = 0.0002;
    public static double f = 0.0011;
    public static double lp = 0.02, li = 0, ld = 0.0002;
    public static double lf = 0.14;

    // Target Positions
    public static double target = 100;
    public static double ltarget = 50;
    public static double h3target = 0;

    // Conversion Constants
    private final double ticks_in_degree = 2.77;
    private final double ticks_in_mm = 3.20;
    private final double h1ticks_in_mm = 3.434;
    private final double h2ticks_in_mm = 3.434;

    // Position Presets
    double armClosed = 10;
    double armMax = 8800;
    double armCosSus = 5915;
    double armCosJos = 5950;
    double armIntake = 1400;
    double armHangPos1 = 7140;
    double armHangPos2 = 8701;
    double armHang3Down = 0;
    double armHang3Up = 3900;
    private static final double ARM_INTAKE_SPECIMEN = 1600;
    private static final double ARM_RUNG = 2100;
    private static final double ARM_OUTTAKE_RUNG = 1800;
    double liftClosed = 10;
    double liftMax = 1000;
    double liftCosSus = 1600;
    double liftCosJos = 500;

    double clesteDeschis = 0.3;
    double clesteInchis = 0;
    double servoTras = 0.7;
    double servoRetras = 0.9;

    // Fudge Factor
    final double FUDGE_FACTOR = 1000;
    double armPositionFudgeFactor;

    // Encoder Error Constants
    private final double ARM_ENCODER_ERROR = 100;
    private final double LIFT_ENCODER_ERROR = 10;

    private int cnt_a = 0;
    private int cnt_b = 0;
    private int cnt_x = 0;
    private int cnt_y = 0;
    private int cnt_right = 0;

    // Timing and State
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;
    private boolean shouldVibrate = true;  // New flag to control vibration

    private static final double LIFT_MIN = 50;
    private static final double LIFT_MAX_EXT = 1570;
    private static final double ARM_MIN = 666;
    private static final double ARM_MIN_FOR_MAX_LIFT = 1100;

    // Toggle Button Readers
    private ToggleButtonReader aToggle;
    private ToggleButtonReader bToggle;
    private ToggleButtonReader yToggle;
    private ToggleButtonReader xToggle;
    private ToggleButtonReader rightToggle;

    @Override
    public void init() {
        // Initialize PID Controllers
        controller = new PIDController(p, i, d);
        lcontroller = new PIDController(lp, li, ld);

        // Set up telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        motor_stanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        fata_dreapta = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        cleste = hardwareMap.get(Servo.class, "cleste");
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");

        // Configure motor directions
        motor_stanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
        fata_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spate_stanga.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize fudge factor
        armPositionFudgeFactor = 0;

        // Configure bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize GamepadEx and ToggleButtonReaders
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);
        aToggle = new ToggleButtonReader(toolOp, GamepadKeys.Button.A);
        bToggle = new ToggleButtonReader(toolOp, GamepadKeys.Button.B);
        yToggle = new ToggleButtonReader(toolOp, GamepadKeys.Button.Y);
        xToggle = new ToggleButtonReader(toolOp, GamepadKeys.Button.X);
        rightToggle = new ToggleButtonReader(driverOp, GamepadKeys.Button.DPAD_RIGHT);
    }

    @Override
    public void loop() {
        // Gamepad initialization
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        // Update toggle states
        aToggle.readValue();
        bToggle.readValue();
        yToggle.readValue();
        xToggle.readValue();
        rightToggle.readValue();

        // Update PID parameters
        controller.setPID(p, i, d);
        lcontroller.setPID(lp, li, ld);

        // Get current positions with simulated encoder error bounds
        int rawArmPos = motor_stanga.getCurrentPosition();
        int rawLiftPos = motor_glisiere.getCurrentPosition();

        // Define position ranges due to encoder errors (for telemetry and awareness)
        double armPosMin = rawArmPos - ARM_ENCODER_ERROR;
        double armPosMax = rawArmPos + ARM_ENCODER_ERROR;
        double liftPosMin = rawLiftPos - LIFT_ENCODER_ERROR;
        double liftPosMax = rawLiftPos + LIFT_ENCODER_ERROR;
        double armPos = rawArmPos;
        double liftPos = rawLiftPos;

        // Calculate PID and feedforward
        double pid = controller.calculate(armPos, target + armPositionFudgeFactor);
        double lpid = lcontroller.calculate(liftPos, ltarget);
        double ff = Math.cos(Math.toRadians((target + armPositionFudgeFactor) / ticks_in_degree)) * f;
        double lff = lf;
        double power = pid + ff;
        double lpower = lpid + lff;

        // Drivetrain control
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // Set motor powers
        fata_stanga.setPower(frontLeftPower);
        spate_stanga.setPower(backLeftPower);
        fata_dreapta.setPower(frontRightPower);
        spate_dreapta.setPower(backRightPower);
        motor_glisiere.setPower(lpower);
        motor_stanga.setPower(power);

        // Update fudge factor
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

        // Lift manual control
        if (gamepad2.right_bumper) {
            ltarget += 15;
        }
        if (gamepad2.left_bumper) {
            ltarget -= 15;
        }

        // Arm and lift presets without intake mode
        if (gamepad2.dpad_left) {
            target = ARM_MIN;
            ltarget = LIFT_MIN;
        }
        if (gamepad2.dpad_right) {
            target = armClosed;
            ltarget = liftClosed;
        }
        if (gamepad2.dpad_up) {
            target = armCosSus;
        }
        if (gamepad2.dpad_up && armPos > 4000) {
            ltarget = liftCosSus;
        }
        if (gamepad2.dpad_down) {
            target = armCosJos;
            ltarget = liftClosed;
        }
        if (rightToggle.wasJustPressed()) {
            if (cnt_right % 2 == 0) {
                target = armHangPos1;
            } else {
                target = armHangPos2;
            }
            cnt_right++;
        }
        if (gamepad1.dpad_left) {
            target = armClosed;
        }
        if (gamepad1.dpad_up) {
            h3target = armHang3Up;
        }
        if (gamepad1.dpad_down) {
            h3target = armHang3Down;
        }

        // Enforce limits
        if (rawLiftPos > LIFT_MAX_EXT) {
            ltarget = ltarget - abs(LIFT_MAX_EXT - ltarget);
        }
        if (rawArmPos < 0) {
            target = 0;
        }
        if(rawArmPos < 4500 && ltarget > 1200 ){
            ltarget = 1200;
        }

        // Servo controls with toggle readers
        if (aToggle.wasJustPressed()) {
            if (cnt_a % 2 == 0) {
                cleste.setPosition(clesteDeschis);
            } else {
                cleste.setPosition(clesteInchis);
                gamepad1.rumble(1000);
            }
            cnt_a++;
        }
        if (bToggle.wasJustPressed()) {
            if (cnt_b % 2 == 0) {
                servoRotire.setPosition(servoTras);
            } else {
                servoRotire.setPosition(servoRetras);
            }
            cnt_b++;
        }
        if (yToggle.wasJustPressed()) {
            target = ARM_INTAKE_SPECIMEN;
            cnt_y++;
        }
        if (xToggle.wasJustPressed()) {
            if (cnt_x % 2 == 0) {
                target = ARM_OUTTAKE_RUNG;
            } else {
                target = ARM_RUNG;
            }
            cnt_x++;
        }

        // Check for options button to stop vibration
        if (gamepad2.options) {
            shouldVibrate = false;
        }

        // Vibration control with flag
        if (shouldVibrate && looptime > 75) {
            gamepad1.rumble(2000);
            gamepad2.rumble(2000);
        }

        // Update timing
        looptime = getRuntime();
        cycletime = looptime - oldtime;
        oldtime = looptime;

        // Telemetry with encoder error ranges
        telemetry.addLine("PETUNIX");
        telemetry.addData("Arm Target", target);
        telemetry.addData("Arm Pos", rawArmPos);
        telemetry.addData("Lift Target", ltarget);
        telemetry.addData("Lift Pos", rawLiftPos);
        telemetry.addLine("ROBOPEDA");
        telemetry.addLine("CIUPA, LUCAS & GEORGE");
        telemetry.addLine("PETUNIX");
        telemetry.update();
    }
}