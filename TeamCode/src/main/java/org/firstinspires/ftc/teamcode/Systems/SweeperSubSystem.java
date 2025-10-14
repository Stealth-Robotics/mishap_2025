package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SweeperSubsystem {
    HardwareMap hardwareMap;
    CRServo leftSweeper;
    CRServo rightSweeper;

    public boolean isSweeperOn() {
        return leftSweeper.getPower() != 0 || rightSweeper.getPower() != 0;
    }

    public SweeperSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftSweeper = hardwareMap.get(CRServo.class, "left_sweeper_servo");
        rightSweeper = hardwareMap.get(CRServo.class, "right_sweeper_servo");
        rightSweeper.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void startIntake() {
        leftSweeper.setPower(1);
        rightSweeper.setPower(1);
    }

    public void stopIntake() {
        leftSweeper.setPower(0);
        rightSweeper.setPower(0);
    }

    public void reverseIntake() {
        leftSweeper.setPower(-1);
        rightSweeper.setPower(-1);
    }
}
