package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

@TeleOp (name = "_TeleOp Driver Only", group = "Main")
public class TeleOpSingleOp extends LinearOpMode {

    private boolean robotCentric = false;
    private boolean slowMode = false;
    private boolean shootIt = false;
    private boolean autoAim = false;
    private RobotSystem robotSystem;
    private TelemetryManager telemetryM;

    @Override
    public void runOpMode() throws InterruptedException {
        robotSystem = new RobotSystem(hardwareMap, telemetry);
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        Follower follower = robotSystem.getFollower();
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            // Robot update handles all message pumps including telemetry
            robotSystem.update();
            boolean isShootReady = robotSystem.getShootReady();

            if (gamepad1.rightStickButtonWasPressed()) {
                slowMode = !slowMode;
            }

            if (gamepad1.backWasPressed()) {
                robotSystem.toggleLimelightPipeline();
            }

            if (gamepad1.xWasPressed()) {
                autoAim = true;
            } else if (gamepad1.xWasReleased()) {
                autoAim = false;
            }

            // see drive method for more controller mappings
            robotSystem.drive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    slowMode,
                    robotCentric,
                    autoAim);

            if (gamepad1.yWasPressed()) {
                robotSystem.resetIMU();
            }

            if (gamepad1.aWasPressed()) {
                shootIt = !shootIt;
                if (shootIt && !robotSystem.getShootReady()) {
                    robotSystem.setReadyShoot();
                }
            }

            if (shootIt) {
                if (isShootReady) {
                    robotSystem.shoot();
                    shootIt = false;
                }

                if (gamepad1.rightBumperWasPressed()) {
                    robotSystem.startIntake();
                } else if (gamepad1.rightBumperWasReleased()) {
                    robotSystem.stopIntake();
                }

                if (gamepad1.bWasPressed()) {
                    robotSystem.incrementSpindexerSlot();
                }

                if (gamepad1.leftBumperWasPressed()) {
                    robotSystem.reverseIntake();
                } else if (gamepad1.leftBumperWasReleased()) {
                    robotSystem.stopIntake();
                }

                if (gamepad1.dpadUpWasPressed()) {
                    robotSystem.increaseShooterRpmFar();
                }

                if (gamepad1.dpadDownWasPressed()) {
                    robotSystem.decreaseShooterRpmFar();
                }

                // Right and left sticks swapped for
                if (gamepad1.leftStickButtonWasPressed()) {
                    robotCentric = !robotCentric;
                }

                if (gamepad1.dpadLeftWasPressed()) {
                    robotSystem.increaseSpindexer();
                }

                if (gamepad1.dpadRightWasPressed()) {
                    robotSystem.decreaseSpindexer();
                }

                if (gamepad1.left_trigger > .2) {
                    robotSystem.setShooterTargetRange(false);
                } else if (gamepad1.right_trigger > .2) {
                    robotSystem.setShooterTargetRange(true);
                }
            }
        }
    }

    /**
     * Wait for the start button to be pressed be careful with this method
     * the robot is in a hot state even if the start button is not pressed
     */
    @Override
    public void waitForStart(){
        while (!isStarted()) {
            robotSystem.update();
            /// DO additional work here like limelight code to check for the
            /// the Motif values
        }
    }


}
