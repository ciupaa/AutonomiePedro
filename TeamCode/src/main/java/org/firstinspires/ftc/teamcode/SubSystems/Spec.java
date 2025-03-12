package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.teamcode.Autonomie.PIDControllerWrapper;

public class Spec extends Subsystem {
    // Singleton pattern (BOILERPLATE)
    public static final Spec INSTANCE = new Spec();
    private Spec() {
        // Private constructor to enforce singleton pattern
    }

    // USER CODE
    public MotorEx spec;
    public PIDControllerWrapper scontroller = new PIDControllerWrapper(
            new PIDController(0.01, 0, 0),
            0,
            20
    );
    public String name = "spec";

    public Command closed() {
        return new RunToPosition(
                spec,    // MOTOR TO MOVE
                0,                // TARGET POSITION, IN TICKS
                scontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command specClosed() {
        return new RunToPosition(
                spec,    // MOTOR TO MOVE
                10,                // TARGET POSITION, IN TICKS
                scontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command Up() {
        return new RunToPosition(
                spec,    // MOTOR TO MOVE
                2000,                // TARGET POSITION, IN TICKS
                scontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command Intake() {
        return new RunToPosition(
                spec,    // MOTOR TO MOVE
                10,                // TARGET POSITION, IN TICKS
                scontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }



    @Override
    public void initialize() {
        spec = new MotorEx(name);
        spec.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(spec, scontroller, this);

    }
}