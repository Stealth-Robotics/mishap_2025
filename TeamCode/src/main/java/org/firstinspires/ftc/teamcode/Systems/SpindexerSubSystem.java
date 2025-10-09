package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpindexerSubSystem {
    private final DcMotorEx spindexer;

    public SpindexerSubSystem(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");
    }

    public void spin(double power) {
        spindexer.setPower(power);
    }
    public double getPosition() {
        return spindexer.getCurrentPosition();
    }
}
