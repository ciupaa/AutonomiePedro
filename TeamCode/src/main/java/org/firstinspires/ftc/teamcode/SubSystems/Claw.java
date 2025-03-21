package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;

public class Claw extends Subsystem {
    // BOILERPLATE
    public static final Claw INSTANCE = new Claw();
    private Claw() { }

    // USER CODE
    public Servo cleste;
    
    public String name = "cleste";

    public Command open() {
        return new ServoToPosition(cleste, // SERVO TO MOVE
                0.4, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(cleste, // SERVO TO MOVE
                0, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command autoOpen() {
        return new ServoToPosition(cleste, // SERVO TO MOVE
                0.65, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        cleste = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }

}
