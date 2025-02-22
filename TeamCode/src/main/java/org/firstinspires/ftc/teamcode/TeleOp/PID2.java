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
    private PIDController hangPid;

    // PID Constants (Configurable via Dashboard)
    public static double ARM_P = 0.0055, ARM_I = 0, ARM_D = 0.0002, ARM_F = 0.0011;
    public static double LIFT_P = 0.01, LIFT_I = 0, LIFT_D = 0.0002, LIFT_F = 0.14;
    public static double HANG_P = 0, HANG_I = 0, HANG_D = 0, HANG_F = 0;

    // Target Positions
    private double armTarget = 100;
    private double liftTarget = 20;
    private double hangTarget = 0;

    // Conversion Constants
    private static final double ARM_TICKS_PER_DEGREE = 2.77;
    private static final double LIFT_TICKS_PER_MM = 3.20;
    private static final double HANG_TICKS_PER_MM = 3.434;
    private static final double FUDGE_FACTOR = 250;

    // Hardware Declarations
    private DcMotorEx armMotor;
    private DcMotorEx liftMotor;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx hangMotor1, hangMotor2;
    private Servo clawServo, rotationServo;
    private CRServo hangServo1, hangServo2;

    // Gamepads
    private GamepadEx driverGamepad;
    private GamepadEx toolGamepad;

    // Position Presets
    private static final double ARM_CLOSED = 10;
    private static final double ARM_MAX = 1000;
    private static final double ARM_COS_SUS = 5950;
    private static final double ARM_COS_JOS = 5950;
    private static final double ARM_INTAKE = 1400;
    private static final double ARM_HANG_POS1 = 7140;
    private static final double ARM_HANG_POS2 = 8701;
    private static final double HANG_DOWN = 10;
    private static final double HANG_UP = 100;

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

    @Override
    public void init() {
        // Initialize PID Controllers
        armPid = new PIDController(ARM_P, ARM_I, ARM_D);
        liftPid = new PIDController(LIFT_P, LIFT_I, LIFT_D);
        hangPid = new PIDController(HANG_P, HANG_I, HANG_D);

        // Set up telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize hardware
        initializeMotors();
        initializeServos();
        configureMotors();

        // Initialize gamepads
        driverGamepad = new GamepadEx(gamepad1);
        toolGamepad = new GamepadEx(gamepad2);

        // Configure bulk caching
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
        frontLeft = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        frontRight = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        backLeft = hardwareMap.get(DcMotorEx.class, "spate_stanga");
        backRight = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        hangMotor1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hangMotor2 = hardwareMap.get(DcMotorEx.class, "hang2");
    }

    private void initializeServos() {
        clawServo = hardwareMap.get(Servo.class, "cleste");
        rotationServo = hardwareMap.get(Servo.class, "servoRotire");
        hangServo1 = hardwareMap.get(CRServo.class, "hang31");
        hangServo2 = hardwareMap.get(CRServo.class, "hang32");
    }

    private void configureMotors() {
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        hangPid.setPID(HANG_P, HANG_I, HANG_D);
    }

    private void updatePositions() {
        int armPos = armMotor.getCurrentPosition();
        int liftPos = liftMotor.getCurrentPosition();
        int hangPos = hangMotor1.getCurrentPosition();

        double armPidOutput = armPid.calculate(armPos, armTarget + armFudgeFactor);
        double liftPidOutput = liftPid.calculate(liftPos, liftTarget);
        double hangPidOutput = hangPid.calculate(hangPos, hangTarget);

        double armFF = Math.cos(Math.toRadians((armTarget + armFudgeFactor) / ARM_TICKS_PER_DEGREE)) * ARM_F;
        double liftFF = LIFT_F;
        double hangFF = HANG_F;

        armMotor.setPower(armPidOutput + armFF);
        liftMotor.setPower(liftPidOutput + liftFF);
        hangMotor1.setPower(hangPidOutput + hangFF);
        hangMotor2.setPower(hangPidOutput + hangFF); // Sync with hangMotor1
    }

    private void updateDriveTrain() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

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
            armTarget = ARM_CLOSED;
        }
        if (gamepad1.dpad_up) {
            hangTarget = HANG_UP;
        }
        if (gamepad1.dpad_down) {
            hangTarget = HANG_DOWN;
        }

        // Claw control
        if (gamepad2.a) {
            clawServo.setPosition(clawServo.getPosition() == CLAW_CLOSED ? CLAW_OPEN : CLAW_CLOSED);
        }

        // Rotation servo
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
        telemetry.addLine("PETUNIX");
        telemetry.addData("pos arm", armMotor.getCurrentPosition());
        telemetry.addData("lift pos", liftMotor.getCurrentPosition());
        telemetry.addData("hang3 pos", hangMotor1.getCurrentPosition());
        telemetry.addLine("ROBOPEDA");
        telemetry.addLine("CIUPA, LUCAS & GEORGE");
        telemetry.addLine("PETUNIX");
        telemetry.update();
    }

    private void updateLoopTiming() {
        loopTime = getRuntime();
        lastLoopTime = loopTime;
    }
}