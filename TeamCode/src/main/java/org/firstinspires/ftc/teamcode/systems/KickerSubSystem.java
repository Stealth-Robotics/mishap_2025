package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KickerSubSystem {
    Servo kickerServo;
    private static final double KICK = 0.5;
    private static final double KICK_READY = 0.2;


    public KickerSubSystem(HardwareMap hardwareMap) {
        kickerServo = hardwareMap.get(Servo.class, "kicker_servo");
    }

    public void slapItBack(){
        kickerServo.setPosition(KICK_READY);
    }

    public void kick() {
        kickerServo.setPosition(KICK);
    }
    public double getPosition(){
        return kickerServo.getPosition();
    }
}
