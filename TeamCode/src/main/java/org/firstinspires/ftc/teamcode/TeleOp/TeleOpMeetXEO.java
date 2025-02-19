package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Config
@TeleOp
public class TeleOpMeetXEO extends OpMode {
    private PIDController lift;
    public static double lp = 0.01, li = 0, ld = 0.0002;
    public static double lf = 0.14;

    public static double ltarget = 0 ;

    private final double ticks_in_mm = 3.20;
    private DcMotorEx motor_glisiere;
    public DcMotor motor_stanga;

    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_dreapta;
    private DcMotorEx spate_stanga;

    private Servo servoRotire;
    private Servo cleste;

    private DcMotorEx hang1 = null;
    private DcMotorEx hang2 = null;

    double liftClosed = 10;
    double liftMax = 1000;
    double liftCosSus = 1600;
    double liftCosJos = 500;

    double armHangPos1 = 7140;
    double armHangPos2 = 8701;



    double clesteDeschis = 0.6;
    double clesteInchis = 0.93;
    double servoTras = 0.6;
    double servoRetras = 0.4;


    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 71.2 // This is the exact gear ratio of the gobilda 60rpm motor
                    * 9.6 // This is the external gear reduction
                    * 1/360.0; // we want ticks per degree, not per rotation


    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double cosSusBrat = 115 * ARM_TICKS_PER_DEGREE;
    final double cosJosBrat = 90 * ARM_TICKS_PER_DEGREE;
    final double hang = 140 * ARM_TICKS_PER_DEGREE;
    final double intake = 25 * ARM_TICKS_PER_DEGREE;
    final double FUDGE_FACTOR = 25 * ARM_TICKS_PER_DEGREE;
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;



    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;


    @Override
    public void init() {
        lift = new PIDController(lp, li, ld);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor_stanga = hardwareMap.get(DcMotor.class, "motor_stanga");
        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");

        fata_dreapta = hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");

        cleste = hardwareMap.get(Servo.class, "cleste");
        servoRotire = hardwareMap.get(Servo.class, "servoRotire");

        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);

        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fata_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spate_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        hang2.setDirection(DcMotorSimple.Direction.REVERSE);

        ((DcMotorEx) motor_stanga).setCurrentAlert(5, CurrentUnit.AMPS);


        motor_stanga.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_stanga.setTargetPosition(0);
        motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_stanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        lift.setPID(lp, li, ld);
        int liftPos = motor_glisiere.getCurrentPosition();
        double lpid = lift.calculate(liftPos, ltarget);
        double lff = lf;
        double lpower = lpid + lff;
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x; // Counteract imperfect strafing
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

        motor_glisiere.setPower(lpower);

        armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

        if(gamepad2.a){
            cleste.setPosition(clesteInchis);
        }
        if(gamepad2.x){
            cleste.setPosition(clesteDeschis);
        }
        if(gamepad2.b){
            servoRotire.setPosition(servoRetras);
        }
        if(gamepad2.y){
            servoRotire.setPosition(servoTras);
        }

        if (gamepad2.right_bumper) {
            ltarget += 20;
        }
        if (gamepad2.left_bumper) {
            ltarget -= 20;
        }

        if (gamepad2.dpad_left) {
            armPosition = intake;
            ltarget = liftClosed;
        }
        if (gamepad2.dpad_right) {
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
            ltarget = liftClosed;
        }

        if (gamepad2.dpad_up) {
            armPosition = cosSusBrat;
        }
        if (gamepad2.dpad_up && motor_stanga.getCurrentPosition() > 4000) {
            ltarget = liftCosSus;
        }

        if (gamepad2.dpad_down) {
            armPosition = cosJosBrat;
            ltarget = liftClosed;
        }

        if (gamepad1.dpad_right) {
            armPosition = armHangPos1;
        }
        if (gamepad1.dpad_left) {
            armPosition = armHangPos2;
        }
        if(gamepad1.a){
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
        }

        if (liftPos > liftCosSus) {
            ltarget = ltarget - abs(liftCosSus - ltarget);
        }
        if (liftPos < 20) {
            ltarget = ltarget + abs(0 - ltarget);
        }

        if (armPosition < ARM_COLLAPSED_INTO_ROBOT) {
            armPosition = ARM_COLLAPSED_INTO_ROBOT;
        }
        if (armPosition < 45 * ARM_TICKS_PER_DEGREE && ltarget > 1200) {
            ltarget = 1200;
        }

        motor_stanga.setTargetPosition((int) (armPosition + armPositionFudgeFactor ));
        ((DcMotorEx) motor_stanga).setVelocity(4500); // Maximum velocity
        motor_stanga.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        looptime = getRuntime();
        cycletime = looptime-oldtime;
        oldtime = looptime;
    }
}
