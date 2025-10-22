package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpindexerSubSystem {
    private final DcMotorEx spindexer;

    public static final double TIKS_PER_REVOLUTION = 537.6;

    public SpindexerSubSystem(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");
        resetPosition();
    }


    public void spin(double power) {
        spindexer.setPower(power);
    }

    public void resetPosition() {
        spindexer.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void runSpeeeed(double power) {
        spindexer.setPower(power);
    }

    //hi

    public int getPosition() {
        return spindexer.getCurrentPosition();
    }
}
