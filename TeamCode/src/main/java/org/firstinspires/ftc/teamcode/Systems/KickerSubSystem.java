package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KickerSubsystem {
    Servo kickerServo;
    private static final double KICKER_UP = 0.5;
    private static final double KICKER_DOWN = 0.2;


    public KickerSubsystem(HardwareMap hardwareMap) {
        kickerServo = hardwareMap.get(Servo.class, "kicker_servo");
    }

    public void kickIt() {

        kickerServo.setPosition(KICKER_UP);
    }

    public void kickReady() {
        kickerServo.setPosition(KICKER_DOWN);
    }

    public boolean isKickReady() {
        return kickerServo.getPosition() == KICKER_DOWN;
    }
}
