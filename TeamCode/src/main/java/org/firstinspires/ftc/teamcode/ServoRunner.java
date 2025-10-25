package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Servo Runner", group="Tests")
public class ServoRunner extends LinearOpMode {
    private boolean isIntake = false;
    private boolean isOuttake = false;
    private boolean isHoodOpen = false;

    private double headingOffset = 0.0;

    private double intakeMinRage = .00;

    private double intakeMaxRange = .59;

    private double curShootPower = .80;


    @Override
    public void runOpMode() throws InterruptedException {


        Servo hopperServo = hardwareMap.get(Servo.class, "hood_servo");
        CRServo servoIntakeRight = hardwareMap.get(CRServo.class, "left_sweeper_servo");
        CRServo servoIntakeLeft = hardwareMap.get(CRServo.class, "right_sweeper_servo");
        Servo servoFlipper = hardwareMap.get(Servo.class, "kicker_servo");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "right_shoot_motor");
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "left_shoot_motor");
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);


        servoIntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        waitForStart();
        telemetry.addLine("Started");
        telemetry.update();
        servoFlipper.setPosition(.2);
        while (opModeIsActive()) {


            telemetry.addLine("Running");
            telemetry.addData("Hood High:", intakeMaxRange);
            telemetry.addData("Hood Low:", intakeMinRage);
            telemetry.addData("Shoot power:", gamepad1.right_trigger * curShootPower);

            if (gamepad1.right_trigger > 0.001) {
                rightShooter.setPower(gamepad1.right_trigger * curShootPower);
                leftShooter.setPower(gamepad1.right_trigger  * curShootPower );
            }
            else {

                rightShooter.setPower(0);
                leftShooter.setPower(0);
            }

            if (gamepad1.rightBumperWasPressed()) {
                if (isIntake) {
                    isIntake = false;
                    hopperServo.setPosition(intakeMinRage);
                    servoIntakeRight.setPower(0);
                    servoIntakeLeft.setPower(0);
                } else {
                    hopperServo.setPosition(intakeMaxRange);
                    servoIntakeRight.setPower(1);
                    servoIntakeLeft.setPower(1);
                    isIntake = true;
                }
            }

            if (gamepad1.leftBumperWasPressed()) {
                if (isOuttake) {
                    servoIntakeRight.setPower(0);
                    servoIntakeLeft.setPower(0);
                    isOuttake = false;
                } else {
                    isOuttake = true;
                    servoIntakeRight.setPower(-1);
                    servoIntakeLeft.setPower(-1);
                }
            }

            if (gamepad1.yWasPressed())
            {
                servoFlipper.setPosition(.5);
            }
            if (gamepad1.yWasReleased()) {
                servoFlipper.setPosition(.2);
            }

            if (gamepad1.dpadUpWasPressed())
            {
                intakeMinRage += .01;
            }

            if (gamepad1.dpadDownWasPressed())
            {
                intakeMinRage -= .01;
            }

            if (gamepad1.dpadLeftWasPressed())
            {
                intakeMaxRange -= .01;
            }
            if (gamepad1.dpadRightWasPressed())
            {
                intakeMaxRange += .01;
            }

            if (gamepad1.aWasPressed()) {
                if (isHoodOpen) {
                    hopperServo.setPosition(intakeMinRage);
                    isHoodOpen = false;
                } else {
                    hopperServo.setPosition(intakeMaxRange);
                    isHoodOpen = true;
                }
            }

            telemetry.update();
        }
    }
}

