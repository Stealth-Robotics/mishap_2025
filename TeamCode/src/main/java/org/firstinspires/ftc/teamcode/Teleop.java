package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Systems.RobotSystem;

@TeleOp (name = "_TeleOp", group = "Main")
public class Teleop extends LinearOpMode {

    private RobotSystem robotSystem;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robotSystem = new RobotSystem(hardwareMap);

    }
}
