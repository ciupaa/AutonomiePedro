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
@TeleOp
public class PidSpecimenSiSample extends OpMode {

    // Hardware Declarations
    private DcMotorEx motor_stanga; // the arm motor
    private DcMotorEx motor_glisiere;
    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_stanga;
    private DcMotorEx spate_dreapta;
    private DcMotorEx hang1 = null;
    private DcMotorEx hang2 = null;
    private Servo servoRotire;
    private Servo cleste;
    private CRServo hang31;
    private CRServo hang32;

    // PID Controllers
    private PIDController controller;
    private PIDController lcontroller;
    private PIDController hang1pid;
    private PIDController hang2pid;

    // PID Tuning Parameters
    public static double p = 0.0055, i = 0, d = 0.0002;
    public static double f = 0.0011;
    public static double lp = 0.01, li = 0, ld = 0.0002;
    public static double lf = 0.14;
    public static double h1p = 0, h1i = 0, h1d = 0;
    public static double h1f = 0;
    public static double h2p = 0, h2i = 0, h2d = 0;
    public static double h2f = 0;

    // Target Positions
    public static double target = 100;
    public static double ltarget = 20;
    public static double h3target = 0;

    // Conversion Constants
    private final double ticks_in_degree = 2.77;
    private final double ticks_in_mm = 3.20;
    private final double h1ticks_in_mm = 3.434;
    private final double h2ticks_in_mm = 3.434;

    // Position Presets
    double armClosed = 10;
    double armCosSus = 5950;
    double armCosJos = 5950;
    double armIntake = 1400;
    double armIntakeSpecimen = 2000;
    double armRung = 2100;
    double armOutTakeRung = 2000;
    double armHangPos1 = 7140;
    double armHangPos2 = 8701;
    double armHang3Down = 10;
    double armHang3Up = 100;

    double liftClosed = 10;
    double liftMax = 1000;
    double liftCosSus = 1600;
    double liftCosJos = 500;

    double clesteDeschis = 0.6;
    double clesteInchis = 1;
    double servoTras = 0.4;
    double servoRetras = 0.6;

    // Fudge Factor
    final double FUDGE_FACTOR = 250;
    double armPositionFudgeFactor;

    // Timing and State
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;


    @Override
    public void init() {
        // Initialize PID Controllers
        controller = new PIDController(p, i, d);
        lcontroller = new PIDController(lp, li, ld);
        hang1pid = new PIDController(h1p, h1i, h1d);
        hang2pid = new PIDController(h2p, h2i, h2d);

        // Set up telemetry
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

        // Initialize fudge factor
        armPositionFudgeFactor = 0;

        // Configure bulk caching
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        // Gamepad initialization
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        // Update PID parameters
        controller.setPID(p, i, d);
        lcontroller.setPID(lp, li, ld);
        hang1pid.setPID(h1p, h1i, h1d);
        hang2pid.setPID(h2p, h2i, h2d);

        // Get current positions
        int armPos = motor_stanga.getCurrentPosition();
        int liftPos = motor_glisiere.getCurrentPosition();
        int hang1Pos = hang1.getCurrentPosition();
        int hang2Pos = hang2.getCurrentPosition();

        // Calculate PID and feedforward
        double pid = controller.calculate(armPos, target + armPositionFudgeFactor);
        double lpid = lcontroller.calculate(liftPos, ltarget);
        double ff = Math.cos(Math.toRadians((target + armPositionFudgeFactor) / ticks_in_degree)) * f;
        double lff = lf;
        double power = pid + ff;
        double lpower = lpid + lff;

        // Hang motors sync
        double leaderPID = hang1pid.calculate(hang1Pos, h3target);
        double leaderFF = h1f;
        double leaderPower = leaderPID + leaderFF;
        hang1.setPower(leaderPower);
        hang2.setPower(leaderPower);

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

        // Hang servos control
        hang31.setPower(gamepad1.right_trigger + (-gamepad1.left_trigger));
        hang32.setPower(gamepad1.right_trigger + (-gamepad1.left_trigger));

        // Update fudge factor
        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

        // Lift manual control
        if (gamepad2.right_bumper) {
            ltarget += 20;
        }
        if (gamepad2.left_bumper) {
            ltarget -= 20;
        }

        // Arm and lift presets
        if (gamepad2.dpad_left) {
            target = armIntake;
            ltarget = liftClosed;
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

        //SPECIMEN
        if(gamepad2.y){
            target = armIntakeSpecimen;
        }
        if(gamepad2.y && target == armIntakeSpecimen){
            target = armRung;
        }
        if(gamepad2.x && target == armRung){
            target = armOutTakeRung;
            if(target == armOutTakeRung) {
                cleste.setPosition(clesteDeschis);
                gamepad1.rumble(1000);
            }
        }



        //ASCENT 2
        if (gamepad1.dpad_right) {
            target = armHangPos1;
        }
        if (gamepad1.dpad_right && armPos > 8600) {
            target = armHangPos2;
        }
        if (gamepad1.dpad_left && armPos > 8600) {
            target = armClosed;
        }

        // ASCENT 3
        if(gamepad1.dpad_up){
            h3target = armHang3Up;
        }
        if(gamepad1.dpad_down){
            h3target = armHang3Down;
        }

        // Enforce limits
        if (liftPos > liftCosSus) {
            ltarget = ltarget - abs(liftCosSus - ltarget);
        }
        if (liftPos < 20) {
            ltarget = ltarget + abs(0 - ltarget);
        }
        if (armPos < 0) {
            target = 0;
            armPositionFudgeFactor = 0;
        }

        // Servo controls
        if (gamepad2.a) {
            cleste.setPosition(clesteInchis);
            gamepad1.rumble(1000);
        }
        if (gamepad2.a && cleste.getPosition() == clesteInchis) {
            cleste.setPosition(clesteDeschis);
        }
        if (gamepad2.b) {
            servoRotire.setPosition(servoTras);
        }
        if (gamepad2.b && servoRotire.getPosition() == servoTras) {
            servoRotire.setPosition(servoRetras);
        }

        // Update timing
        looptime = getRuntime();
        cycletime = looptime - oldtime;
        oldtime = looptime;

        // Telemetry
        telemetry.addLine("PETUNIX");
        telemetry.addData("pos arm", armPos);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("hang3 pos", hang1Pos);
        telemetry.addLine("ROBOPEDA");
        telemetry.addLine("CIUPA, LUCAS & GEORGE");
        telemetry.addLine("PETUNIX");
        telemetry.update();
    }
}