package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;

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
@TeleOp(name = "PID2 TeleOp")
public class PID2 extends OpMode {
    // PID Controllers
    private PIDController armPid;
    private PIDController liftPid;
    private PIDController hang1Pid;
    private PIDController hang2Pid;

    // PID Constants (Configurable via Dashboard)
    public static double ARM_P = 0.0055, ARM_I = 0, ARM_D = 0.0002, ARM_F = 0.0011;
    public static double LIFT_P = 0.01, LIFT_I = 0, LIFT_D = 0.0002, LIFT_F = 0.14;
    public static double HANG1_P = 0, HANG1_I = 0, HANG1_D = 0, HANG1_F = 0;
    public static double HANG2_P = 0, HANG2_I = 0, HANG2_D = 0, HANG2_F = 0;

    // Target_positions
    private double armTarget = 0;
    private double liftTarget = 0;
    private double hang1Target = 0;
    private int hang2Target = 0;

    // Conversion Constants
    private static final double ARM_TICKS_PER_DEGREE = 2.77;
    private static final double HANG_TICKS_PER_DEGREE = 3.434;
    private static final double LIFT_TICKS_PER_MM = 3.20;
    private static final double FUDGE_FACTOR = 250; // Max adjustment in ticks

    // Hardware Declarations
    private DcMotorEx armMotor;
    private DcMotorEx liftMotor;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx hangMotor1, hangMotor2;
    private Servo clawServo, rotationServo;
    private CRServo hangServo1, hangServo2;

    // Gamepad
    private GamepadEx driverGamepad;
    private GamepadEx toolGamepad;

    // Position Constants
    private static final double ARM_CLOSED = 10;
    private static final double ARM_MAX = 1000;
    private static final double ARM_COS_SUS = 5950;
    private static final double ARM_COS_JOS = 5950;
    private static final double ARM_INTAKE = 1400;
    private static final double ARM_HANG_POS1 = 7140;
    private static final double ARM_HANG_POS2 = 8701;
    private static final double ARM_HANG3_CLOSED = 10;

    private static final double LIFT_CLOSED = 10;
    private static final double LIFT_MAX = 1000;
    private static final double LIFT_COS_SUS = 1600;
    private static final double LIFT_COS_JOS = 500;

    private static final double CLAW_OPEN = 0.6;
    private static final double CLAW_CLOSED = 1.0;
    private static final double ROTATION_EXTENDED = 0.4;
    private static final double ROTATION_RETRACTED = 0.6;

    // State Variables
    private double armFudgeFactor = 0;
    private double loopTime = 0;
    private double lastLoopTime = 0;
    private boolean pidEnabled = true;

    @Override
    public void init() {
        // Initialize PID Controllers
        armPid = new PIDController(ARM_P, ARM_I, ARM_D);
        liftPid = new PIDController(LIFT_P, LIFT_I, LIFT_D);
        hang1Pid = new PIDController(HANG1_P, HANG1_I, HANG1_D);
        hang2Pid = new PIDController(HANG2_P, HANG2_I, HANG2_D);

        // Set up telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        initializeMotors();
        initializeServos();
        configureMotors();

        // Initialize gamepads
        driverGamepad = new GamepadEx(gamepad1);
        toolGamepad = new GamepadEx(gamepad2);

        // Optimize hardware reading
        configureBulkCaching();
    }

    @Override
    public void loop() {
        updatePidParameters();
        updatePositions();
        updateDriveTrain();
        updateMechanisms();
        enforceLimits();
        updateTelemetry();
        updateLoopTiming();
    }

    private void initializeMotors() {
        armMotor = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        liftMotor = hardwareMap.get(DcMotorEx.class, "motor_glisiere");
        hangMotor1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hangMotor2 = hardwareMap.get(DcMotorEx.class, "hang2");

        frontLeft = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        frontRight = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        backLeft = hardwareMap.get(DcMotorEx.class, "spate_stanga");
        backRight = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
    }

    private void initializeServos() {
        clawServo = hardwareMap.get(Servo.class, "cleste");
        rotationServo = hardwareMap.get(Servo.class, "servoRotire");
        hangServo1 = hardwareMap.get(CRServo.class, "hang31");
        hangServo2 = hardwareMap.get(CRServo.class, "hang32");
    }

    private void configureMotors() {
        // Motor directions
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Brake behavior
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void configureBulkCaching() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private void updatePidParameters() {
        armPid.setPID(ARM_P, ARM_I, ARM_D);
        liftPid.setPID(LIFT_P, LIFT_I, LIFT_D);
        hang1Pid.setPID(HANG1_P, HANG1_I, HANG1_D);
        hang2Pid.setPID(HANG2_P, HANG2_I, HANG2_D);
    }

    private void updatePositions() {
        int armPos = armMotor.getCurrentPosition();
        int liftPos = liftMotor.getCurrentPosition();
        int hang1Pos = hangMotor1.getCurrentPosition();
        int hang2Pos = hangMotor2.getCurrentPosition();

        // Calculate PID outputs
        double armPidOutput = armPid.calculate(armPos, armTarget + armFudgeFactor);
        double liftPidOutput = liftPid.calculate(liftPos, liftTarget);
        double hangPidOutput = hang1Pid.calculate(hang1Pos, hang1Target);

        // Calculate feedforward
        double armFF = Math.cos(Math.toRadians((armTarget + armFudgeFactor) / ARM_TICKS_PER_DEGREE)) * ARM_F;
        double liftFF = LIFT_F;
        double hangFF = HANG1_F;

        // Apply power
        armMotor.setPower(armPidOutput + armFF);
        liftMotor.setPower(liftPidOutput + liftFF);
        hangMotor1.setPower(hangPidOutput + hangFF);
        hangMotor2.setPower(hangPidOutput + hangFF); // Sync with hangMotor1
    }

    private void updateDriveTrain() {
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x;   // Strafe
        double rx = gamepad1.right_stick_x; // Rotation

        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    private void updateMechanisms() {
        // Hang servos
        double hangPower = gamepad1.right_trigger - gamepad1.left_trigger;
        hangServo1.setPower(hangPower);
        hangServo2.setPower(hangPower);

        // Arm fudge factor
        armFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger - gamepad2.left_trigger);

        // Lift control
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
            if (armMotor.getCurrentPosition() > 4000) liftTarget = LIFT_COS_SUS;
        }
        if (gamepad2.dpad_down) {
            armTarget = ARM_COS_JOS;
            liftTarget = LIFT_CLOSED;
        }
        if (gamepad1.dpad_right) {
            armTarget = ARM_HANG_POS1;
            if (armMotor.getCurrentPosition() > 8600) armTarget = ARM_HANG_POS2;
        }
        if (gamepad1.dpad_left && armMotor.getCurrentPosition() > 8600) {
            armTarget = ARM_HANG3_CLOSED;
        }

        // Claw control (toggle)
        if (gamepad2.a) {
            clawServo.setPosition(clawServo.getPosition() == CLAW_CLOSED ? CLAW_OPEN : CLAW_CLOSED);
        }

        // Rotation servo (toggle)
        if (gamepad2.b) {
            rotationServo.setPosition(rotationServo.getPosition() == ROTATION_EXTENDED ?
                    ROTATION_RETRACTED : ROTATION_EXTENDED);
        }
    }

    private void enforceLimits() {
        int armPos = armMotor.getCurrentPosition();
        int liftPos = liftMotor.getCurrentPosition();

        if (liftPos > LIFT_COS_SUS) {
            liftTarget -= abs(LIFT_COS_SUS - liftTarget);
        }
        if (liftPos < 20) {
            liftTarget += abs(0 - liftTarget);
        }
        if (armPos < 0) {
            armTarget = 0;
            armFudgeFactor = 0;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("PID Enabled", pidEnabled);
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Lift Position", liftMotor.getCurrentPosition());
        telemetry.addData("Hang1 Position", hangMotor1.getCurrentPosition());
        telemetry.addData("Hang2 Position", hangMotor2.getCurrentPosition());
        telemetry.addData("Fudge Factor", armFudgeFactor);
        telemetry.update();
    }

    private void updateLoopTiming() {
        loopTime = getRuntime();
        lastLoopTime = loopTime;
    }
}