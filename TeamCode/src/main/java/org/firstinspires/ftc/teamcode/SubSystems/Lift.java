package org.firstinspires.ftc.teamcode.SubSystems;

import static com.pedropathing.localization.GoBildaPinpointDriver.EncoderDirection.REVERSED;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.FollowerConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import org.firstinspires.ftc.teamcode.Autonomie.PIDControllerWrapper;
import java.util.Collections;
import java.util.HashSet;

public class Lift extends Subsystem {
    // BOILERPLATE
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE
    public MotorEx motor_glisiere;

    // public PIDFController controller = new PIDFController(new PIDCoefficients(0.005, 0.0, 0.0));
   // public PIDControllerWrapper controller = new PIDControllerWrapper(new PIDController(0.01, 0, 0.0002));

    public PIDControllerWrapper lcontroller = new PIDControllerWrapper(new PIDController(0.01, 0, 0.0002), 0.14, 50);
    public String name = "motor_glisiere";
    public Command toLow() {
        return new RunToPosition(motor_glisiere, // MOTOR TO MOVE
                10, // TARGET POSITION, IN TICKS
                lcontroller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toMiddle() {
        return new RunToPosition(motor_glisiere, // MOTOR TO MOVE
                500, // TARGET POSITION, IN TICKS
                lcontroller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHigh() {
        return new RunToPosition(motor_glisiere, // MOTOR TO MOVE
                1600, // TARGET POSITION, IN TICKS
                lcontroller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    
    @Override
    public void initialize() {
        motor_glisiere = new MotorEx(name);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
