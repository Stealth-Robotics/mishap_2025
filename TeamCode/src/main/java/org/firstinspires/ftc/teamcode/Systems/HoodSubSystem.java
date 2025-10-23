package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodSubSystem {

    Servo hoodServo;
    private static final double HOOD_OPEN_POSITION = 0.29;
    private static final double HOOD_CLOSED_POSITION = 0.02;

    public HoodSubSystem(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hood_servo");
    }

    public void hoodIntake() {
        hoodServo.setPosition(HOOD_OPEN_POSITION);
    }
    public void hoodShoot() {
        hoodServo.setPosition(HOOD_CLOSED_POSITION);
    }
}
