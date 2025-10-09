package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SweeperSubSystem {
    HardwareMap hardwareMap;
    CRServo leftSweeper;
    CRServo rightSweeper;

    public SweeperSubSystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        leftSweeper = hardwareMap.get(CRServo.class, "left_Sweeper_Servo");
        rightSweeper = hardwareMap.get(CRServo.class, "right_Sweeper_Servo");
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
