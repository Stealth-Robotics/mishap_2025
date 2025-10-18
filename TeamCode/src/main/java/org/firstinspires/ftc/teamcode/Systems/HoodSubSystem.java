package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;

public class HoodSubsystem {

    Servo hoodServo;
    private static final double HOOD_OPEN_POSITION = 0.29;
    private static final double HOOD_CLOSED_POSITION = 0.02;

    private double openPosition = HOOD_OPEN_POSITION;
    private double closePosition = HOOD_CLOSED_POSITION;

    private boolean shootPosition = true;



    public HoodSubsystem(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hood_servo");
        hoodShoot();
    }

    public void hoodIntake() {
        shootPosition = false;
        hoodServo.setPosition(openPosition);
    }
    public void hoodShoot() {

        hoodServo.setPosition(closePosition);
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                shootPosition = true;
            }
        }, 200);
    }

    public boolean shootReady()
    {
        return shootPosition;
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
