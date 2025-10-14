package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodSubSystem {

    Servo hoodServo;
    private static final double HOOD_OPEN_POSITION = 0.29;
    private static final double HOOD_CLOSED_POSITION = 0.02;

    private double openPosition = HOOD_OPEN_POSITION;
    private double closePosition = HOOD_CLOSED_POSITION;

    public HoodSubSystem(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hood_servo");
    }

    public void hoodIntake() {
        hoodServo.setPosition(openPosition);
    }
    public void hoodShoot() {
        hoodServo.setPosition(closePosition);
    }

    public boolean shootReady()
    {
        return hoodServo.getPosition() == closePosition;
    }

    public void increaseOpenPosition() {
        openPosition += 0.01;
        if (openPosition > 1.0) {
            openPosition = 1.0;
        }
    }

    public void decrementOpenPosition() {
        openPosition -= 0.01;
        if (openPosition < 0.0) {
            openPosition = 0.0;
        }
    }

    public void increaseClosePosition() {
        closePosition += 0.01;
        if (closePosition > 1.0) {
            closePosition = 1.0;
        }
    }

    public void decrementClosePosition() {
        closePosition -= 0.01;
        if (closePosition < 0.0) {
            closePosition = 0.0;
        }
    }
}
