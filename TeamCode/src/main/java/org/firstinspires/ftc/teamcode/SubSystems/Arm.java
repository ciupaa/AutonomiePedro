package org.firstinspires.ftc.teamcode.SubSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.teamcode.Autonomie.PIDControllerWrapper;

public class Arm extends Subsystem {
    // Singleton pattern (BOILERPLATE)
    public static final Arm INSTANCE = new Arm();
    private Arm() {
        // Private constructor to enforce singleton pattern
    }

    // USER CODE
    public MotorEx motor_stanga;
    public PIDControllerWrapper controller = new PIDControllerWrapper(
            new PIDController(0.0055, 0, 0.0002),
            0.0011,
            200
    );
    public String name = "motor_stanga";

    public Command closed() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                10,              // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }

    public Command intake() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                800,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }

    public Command toHigh() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                5450,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command toHighUp() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                5700,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command toSpecIntake() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                5950,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command specOutTake() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                900,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command specIntake() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                1000,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command hangPos1() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                7140,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }

    public Command specUp() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                1200,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command grabIntake() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                160,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command armBack() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                6750,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command armPick() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                80,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command armOut() {
        return new RunToPosition(
                motor_stanga,    // MOTOR TO MOVE
                80,            // TARGET POSITION, IN TICKS
                controller,      // CONTROLLER TO IMPLEMENT
                this            // IMPLEMENTED SUBSYSTEM
        );
    }


    @Override
    public void initialize() {
        motor_stanga = new MotorEx(name);
    }

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor_stanga, controller, this);
    }
}