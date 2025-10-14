package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubSystem {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;
    public static final double SHOOT_MAX_POWER = 1.0;
    public static final double SHOOT_MIN_POWER = 0.5;

    public static final double SHOOT_DEFAULT_POWER = .9;

    public ShooterSubSystem(HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, "right_shoot_motor");
        leftShooter = hardwareMap.get(DcMotorEx.class, "left_shoot_motor");
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void shoot(double power) {
        if (power > SHOOT_MAX_POWER) {
            power = SHOOT_MAX_POWER;
        } else if (power < SHOOT_MIN_POWER) {
            power = SHOOT_MIN_POWER;
        }

        rightShooter.setPower(power);
        leftShooter.setPower(power);
    }

    public boolean shootReady()
    {
        // TODO: need to get RPM values from motors
        return true;
    }

}
