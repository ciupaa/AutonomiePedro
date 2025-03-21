package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.Autonomie.AutoSampleCopie4.lift1;
import static org.firstinspires.ftc.teamcode.Autonomie.AutoSampleCopie4.lift2;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import org.firstinspires.ftc.teamcode.Autonomie.PIDControllerWrapper;
public class Lift extends Subsystem {
    // Singleton pattern (BOILERPLATE)
    public static final Lift INSTANCE = new Lift();
    private Lift() {
        // Private constructor to enforce singleton pattern
    }

    // USER CODE
    public MotorEx motor_glisiere;
    public PIDControllerWrapper lcontroller = new PIDControllerWrapper(
            new PIDController(0.02, 0, 0.0002),
            0.14,
            70
    );
    public String name = "motor_glisiere";

    public Command closed() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                50,                // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }

    public Command toIntakeSpecimen() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                520,               // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command grab2() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                430,               // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }

    public Command toHigh() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                1600,              // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command intake() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                600,              // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }
    public Command l1() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                lift1,              // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }    public Command l2() {
        return new RunToPosition(
                motor_glisiere,    // MOTOR TO MOVE
                lift2,              // TARGET POSITION, IN TICKS
                lcontroller,       // CONTROLLER TO IMPLEMENT
                this              // IMPLEMENTED SUBSYSTEM
        );
    }

    @Override
    public void initialize() {
        motor_glisiere = new MotorEx(name);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(motor_glisiere, lcontroller, this);

    }
}