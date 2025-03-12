package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class ServoRotire extends Subsystem {
    // BOILERPLATE
    public static final ServoRotire INSTANCE = new ServoRotire();
    private ServoRotire() { }

    // USER CODE
    public Servo servoRotire;
    
    public String name = "servoRotire";

    public Command intake() {
        return new ServoToPosition(servoRotire, // SERVO TO MOVE
                0.9, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command outtake() {
        return new ServoToPosition(servoRotire, // SERVO TO MOVE
                0.7, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command autoOut() {
        return new ServoToPosition(servoRotire, // SERVO TO MOVE
                1, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    @Override
    public void initialize() {
        servoRotire = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }
}
