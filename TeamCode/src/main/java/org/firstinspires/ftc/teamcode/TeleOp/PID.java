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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

import java.util.List;

@TeleOp
public class PID extends OpMode {

    private PIDController controller;
    public static double p = 0.0055, i = 0, d = 0.0002;
    public static double f = 0.0011;

    public static double target = 0 ;

    private final double ticks_in_degree = 2.77;

    private DcMotorEx motor_stanga; //the arm motor

    private PIDController hang1pid;
    public static double h1p = 0, h1i = 0, h1d = 0;
    public static double h1f = 0;

    public static double h1target = 0 ;

    private final double h1ticks_in_degree = 3.434;

    private PIDController hang2pid;
    public static double h2p = 0, h2i = 0, h2d = 0;
    public static double h2f = 0;

    public static int h2target = 0 ;

    private final double h2ticks_in_degree = 3.434;

    private PIDController lcontroller;
    public static double lp = 0.01, li = 0, ld = 0.0002;
    public static double lf = 0.14;

    public static double ltarget = 0 ;

    private final double ticks_in_mm = 3.20;


    private DcMotorEx motor_glisiere;

    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_dreapta;
    private DcMotorEx spate_stanga;

    private Servo servoRotire;
    private Servo cleste;


    private DcMotorEx hang1 = null;
    private DcMotorEx hang2 = null;


    double armClosed = 10;
    double armMax = 1000;


    double armCosSus = 5950;
    double armCosJos = 5950;
    double armIntake = 1400;
    double armHangPos1 = 7140;
    double armHangPos2 = 8701;

    double armHang3Closed = 10;
    double armHang3Open = 100;



    double liftClosed = 10;
    double liftMax = 1000;
    double liftCosSus = 1600;
    double liftCosJos = 500;



    double clesteDeschis = 0.6;
    double clesteInchis = 1;
    double servoTras = 0.4;
    double servoRetras = 0.6;


    double TempTarget = 0;
    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    // --- New variables for PID toggle (gamepad2 L3) ---
    private boolean pidEnabled = true;
    private boolean previousLeftStick = false;

    @Override
    public void init() {

        controller = new PIDController(p, i, d);
        hang1pid = new PIDController(h1p, h1i, h1d);
        hang2pid = new PIDController(h2p, h2i, h2d);
        lcontroller = new PIDController(lp, li, ld);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor_stanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");

        fata_dreapta = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");

        cleste = hardwareMap.get(Servo.class, "cleste");
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");

        motor_stanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);

        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fata_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spate_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        // --- Toggle PID mode for motor_stanga using gamepad2 L3 ---
        if (gamepad2.dpad_left && !previousLeftStick) {
            pidEnabled = !pidEnabled;
        }
        previousLeftStick = gamepad2.dpad_left;

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);
        controller.setPID(p, i, d);
        hang1pid.setPID(h1p, h1i, h1d);
        hang2pid.setPID(h2p, h2i, h2d);
        lcontroller.setPID(lp, li, ld);
        int armPos = motor_stanga.getCurrentPosition();
        int liftPos = motor_glisiere.getCurrentPosition();
        int hang1Pos = hang1.getCurrentPosition();
        int hang2Pos = hang2.getCurrentPosition();

        double pid = controller.calculate(armPos, target);
        double lpid = lcontroller.calculate(liftPos, ltarget);

        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double lff = lf;

        double power = pid + ff;
        double lpower = lpid + lff;

        // --- Syncing hang1 and hang2 using hang1 as the leader ---
        double leaderPID = hang1pid.calculate(hang1Pos, h1target);
        double leaderFF  = h1f; // using the constant feedforward value for hang1
        double leaderPower = leaderPID + leaderFF;
        hang1.setPower(leaderPower);
        hang2.setPower(leaderPower);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        fata_stanga.setPower(frontLeftPower);
        spate_stanga.setPower(backLeftPower);
        fata_dreapta.setPower(frontRightPower);
        spate_dreapta.setPower(backRightPower);

        motor_glisiere.setPower(lpower);

        if (gamepad2.right_bumper) {
            ltarget += 20;
        }
        if (gamepad2.left_bumper) {
            ltarget -= 20;
        }

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

        if (gamepad1.dpad_right) {
            target = armHangPos1;
        }
        if (gamepad1.dpad_left) {
            target = armHangPos2;
        }

        // --- Use PID or manual control for motor_stanga based on pidEnabled ---
        if(pidEnabled) {
            motor_stanga.setPower(power);
        } else {
            motor_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor_stanga.setPower(gamepad2.right_trigger + (-gamepad2.left_trigger));
        }

        // LIMITE



        if (liftPos > liftCosSus) {
            ltarget = ltarget - abs(liftCosSus - ltarget);
        }
        if (liftPos < 20) {
            ltarget = ltarget + abs(0 - ltarget);
        }

        if (armPos < 0) {
            target = 0;
        }

        if (gamepad2.y) {
            servoRotire.setPosition(0.6);
        }
        if (gamepad2.b) {
            servoRotire.setPosition(0.4);
        }

        if (gamepad2.x) {
            cleste.setPosition(clesteDeschis);
        }
        if (gamepad2.a) {
            cleste.setPosition(clesteInchis);
        }

        if (gamepad1.a) {
            target = armClosed;
        }

        looptime = getRuntime();
        cycletime = looptime - oldtime;
        oldtime = looptime;

        telemetry.addData("PID Enabled", pidEnabled);
        telemetry.addData("pos arm", armPos);
        telemetry.addData("lift pos", liftPos);
        telemetry.addData("h1 pos", hang1Pos);
        telemetry.addData("h2 pos", hang2Pos);
        telemetry.update();
    }
}
