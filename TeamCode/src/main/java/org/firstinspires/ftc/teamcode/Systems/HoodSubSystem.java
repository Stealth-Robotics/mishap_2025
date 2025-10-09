package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodSubSystem {

    Servo hoodServo;
    private static final double HOOD_OPEN_POSITION = 0.5;
    private static final double HOOD_CLOSED_POSITION = 0.2;

    public HoodSubSystem(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "intake_servo");
    }

    public void openHood() {
        hoodServo.setPosition(HOOD_OPEN_POSITION);
    }
    public void closeHood() {
        hoodServo.setPosition(HOOD_CLOSED_POSITION);
    }
}
