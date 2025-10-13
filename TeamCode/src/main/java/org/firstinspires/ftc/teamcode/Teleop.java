package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.RobotSystem;

@TeleOp (name = "_TeleOp", group = "Main")
public class Teleop extends LinearOpMode {

    private RobotSystem robotSystem;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robotSystem = new RobotSystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            robotSystem.update();

            robotSystem.getFollower().setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true);
        }

    }
}
