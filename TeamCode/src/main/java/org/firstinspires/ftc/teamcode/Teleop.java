package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "_TeleOp", group = "Main")
public class TeleOp extends LinearOpMode {

    private RobotSystem robotSystem;
    private Boolean isRobotCentric = false;
    private static final double MINSPEED = 0.2;
    private static final double MAXSPEED = 1;

    private boolean slowmode = false;

    private static final Pose START_POSE = new Pose(0, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robotSystem = new RobotSystem(hardwareMap, telemetry);
        Follower follower = robotSystem.getFollower();
        follower.startTeleopDrive();
        follower.setStartingPose(START_POSE);
        robotSystem.resetIMU();
        waitForStart();

        while (opModeIsActive()) {
            robotSystem.update();

            if (gamepad1.yWasPressed())
                robotSystem.resetIMU();

            if (gamepad1.leftStickButtonWasPressed())
                slowmode = !slowmode;

            double curSpeed = slowmode ? MINSPEED : MAXSPEED;

            if (gamepad1.rightStickButtonWasPressed())
                isRobotCentric = !isRobotCentric;

            robotSystem.getFollower().setTeleOpDrive(
                    -gamepad1.left_stick_y * curSpeed,
                    -gamepad1.left_stick_x * curSpeed,
                    -gamepad1.right_stick_x * curSpeed,
                    isRobotCentric);

            if (gamepad1.rightBumperWasPressed()){
                robotSystem.startIntake();
            }

        }

    }
}
