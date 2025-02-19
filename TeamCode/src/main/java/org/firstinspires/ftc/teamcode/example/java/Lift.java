package org.firstinspires.ftc.teamcode.example.java;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.control.coefficients.PIDCoefficients;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import java.util.Collections;
import java.util.HashSet;

public class Lift extends Subsystem {
    // BOILERPLATE
    public static final Lift INSTANCE = new Lift();
    private Lift() { }

    // USER CODE
    public MotorEx motor;

    public PIDController controller = new PIDController(new PIDCoefficients(0.005, 0.0, 0.0));

    public String name = "lift_motor";

    public Command toLow() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                0.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toMiddle() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                500.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHigh() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                1200.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }
    
    @Override
    public void initialize() {
        motor = new MotorEx(name);
    }
}
