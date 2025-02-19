package org.firstinspires.ftc.teamcode.example.java;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Claw extends Subsystem {
    // BOILERPLATE
    public static final Claw INSTANCE = new Claw();
    private Claw() { }

    // USER CODE
    public Servo servo;
    
    public String name = "claw_servo";

    public Command open() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.2, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }
}
