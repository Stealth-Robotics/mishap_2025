package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.RobotSystem;

@TeleOp (name = "_TeleOp Driver Only", group = "Main")
public class TeleOpSingleOp extends LinearOpMode {

    private Boolean robotCentric = false;

    private Boolean slowMode = false;
    private static final double SLOW_MODE_MULTIPLIER = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotSystem robotSystem = new RobotSystem(hardwareMap, telemetry);
        Follower follower = robotSystem.getFollower();
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            robotSystem.update();

            if (gamepad1.leftStickButtonWasPressed()) {
                slowMode = !slowMode;
            }

            double powerMultiplier = slowMode ? SLOW_MODE_MULTIPLIER : 1;
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * powerMultiplier,
                    -gamepad1.left_stick_x * powerMultiplier,
                    -gamepad1.right_stick_x * powerMultiplier,
                    robotCentric);

            if (gamepad1.yWasPressed())
            {
                robotSystem.resetIMU();
            }

            if (gamepad1.aWasPressed()) {
                if (robotSystem.getShootReady()) {
                    telemetry.addLine("Ready to shoot");
                    robotSystem.shoot();
                }else{
                    telemetry.addLine("Not ready to shoot");
                    robotSystem.readyShoot();
                }
            }

            if (gamepad1.rightBumperWasPressed()) {
                robotSystem.startIntake();
            } else if (gamepad1.rightBumperWasReleased()) {
                robotSystem.stopIntake();
            }

            if (gamepad1.bWasPressed()) {
                robotSystem.incrementSpindexerSlot();
            } else if (gamepad1.xWasPressed()) {
                robotSystem.decrementSpindexerSlot();
            }

            if (gamepad1.leftBumperWasPressed()) {
                robotSystem.reverseIntake();
            } else if (gamepad1.leftBumperWasReleased()) {
                robotSystem.stopIntake();
            }

            if (gamepad1.dpadUpWasPressed()) {
                robotSystem.increaseShooterRPM();
            }

            if (gamepad1.dpadDownWasPressed()) {
                robotSystem.decreaseShooterRPM();
            }

            if (gamepad1.rightStickButtonWasPressed()) {
                robotCentric = !robotCentric;
            }

            if(gamepad1.dpadLeftWasPressed()) {
                robotSystem.increaseSpindexer();
            }

            if(gamepad1.dpadRightWasPressed()) {
                robotSystem.decreaseSpindexer();
            }
        }
    }

/*
    @Override
    public void waitForStart(){
        while (!isStarted()) {
            try {
                /// DO additional work here like limelight code to check for the
                /// the Motif values

                // Can print info on what the robot is doing while in init state
                telemetry.addLine("Waiting to start");
                telemetry.update();
                // Can remove if actually doing work
                //noinspection BusyWait
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }
*/

}
