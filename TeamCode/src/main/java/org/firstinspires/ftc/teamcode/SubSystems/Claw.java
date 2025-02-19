package org.firstinspires.ftc.teamcode.SubSystems;

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
    public Servo cleste;
    
    public String name = "cleste";

    public Command open() {
        return new ServoToPosition(cleste, // SERVO TO MOVE
                0.6, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command close() {
        return new ServoToPosition(cleste, // SERVO TO MOVE
                0.93, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        cleste = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }
}
