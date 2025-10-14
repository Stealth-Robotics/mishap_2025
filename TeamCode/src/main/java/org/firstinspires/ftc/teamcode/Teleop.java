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
        Follower follower = robotSystem.getFollower();
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(0, 0, 0));

        while (opModeIsActive()) {
            robotSystem.update();
            if (gamepad1.aWasPressed()) {
                follower.getPoseTracker().getLocalizer().resetIMU();
            }

            if (gamepad1.rightBumperWasPressed())
            {
                robotSystem.startIntake();
            }

            robotSystem.getFollower().setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false);

            if (gamepad1.right_trigger > 0) {
                robotSystem.setSpindexerPower(gamepad1.right_trigger *.5);
            }
            else if (gamepad1.left_trigger > 0) {
                robotSystem.setSpindexerPower(-gamepad1.left_trigger *.2);
            }
            else {
                robotSystem.setSpindexerPower(0);
            }

            if (gamepad1.leftBumperWasPressed()) {
                robotSystem.stopIntake();
            }

            if (gamepad1.xWasPressed()) {
                robotSystem.readyShoot();
            }

            if (gamepad1.bWasPressed()) {
                robotSystem.shoot();
            }

        }

    }
}
