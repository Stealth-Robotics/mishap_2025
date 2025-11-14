package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.systems.SpindexerSubsystem;

@TeleOp(name="Servo Runner", group="Tests")
@Disabled
public class ServoRunner extends LinearOpMode {
    private boolean isIntake = false;
    private boolean isOuttake = false;
    private boolean isHoodOpen = false;

    private double headingOffset = 0.0;

    private double intakeMinRage = .33;

    private double intakeMaxRange = .58;

    private double curShootPower = 1;
    double currentVelocity;
    double maxVelocity = 0.0;

    public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(-.55, -3.6,-.001, -10);  //10, 2,1.2, 1); 8, 4,0.2, 1

    @Override
    public void runOpMode() throws InterruptedException {


        Servo hoodServo = hardwareMap.get(Servo.class, "hood_servo");
        CRServo servoIntakeRight = hardwareMap.get(CRServo.class, "left_sweeper_servo");
        CRServo servoIntakeLeft = hardwareMap.get(CRServo.class, "right_sweeper_servo");
        Servo servoFlipper = hardwareMap.get(Servo.class, "kicker_servo");
        DcMotorEx rightShooter = hardwareMap.get(DcMotorEx.class, "right_shoot_motor");
        DcMotorEx leftShooter = hardwareMap.get(DcMotorEx.class, "left_shoot_motor");

        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");
        //spindexer.resetDeviceConfigurationForOpMode();

        spindexer.setDirection(DcMotorSimple.Direction.REVERSE);
        spindexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set a tolerance for how close to the target is "close enough" (in encoder ticks)
        // This can help prevent oscillations around the target.
        spindexer.setTargetPositionTolerance(2);

        // Default to BRAKE mode for holding position.
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // spindexer.setTargetPosition(0); // Important to set a target after mode change
        //spindexer.(2400);
        //spindexer.setPower(.7);
        spindexer.setVelocity(0);

        // ...// Apply the defined PIDF coefficients to the motor for RUN_TO_POSITION mode
        spindexer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SPINDEXER_PIDF);
        //spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

            telemetry.addData("Spindexer Velocity", spindexer.getVelocity());
            telemetry.addData("Spindixer Position", spindexer.getCurrentPosition());
            telemetry.addData("Spindiexer Mode ", spindexer.getMode());
            telemetry.addData("Spindixer power", spindexer.getPower());
            telemetry.addData("Spindixer Busy", spindexer.isBusy());


            if (gamepad1.xWasPressed()) {
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spindexer.setVelocity(100);
            } else if (gamepad1.xWasReleased())
            {
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spindexer.setVelocity(0); // Or your desired velocity
            }

//            telemetry.addLine("Running");
//            telemetry.addData("Hood High:", intakeMaxRange);
//            telemetry.addData("Hood Low:", intakeMinRage);
            telemetry.addData("Shoot power:", gamepad1.right_trigger * curShootPower);

            currentVelocity = rightShooter.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("max Velocity", maxVelocity);


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
                    hoodServo.setPosition(intakeMinRage);
                    servoIntakeRight.setPower(0);
                    servoIntakeLeft.setPower(0);
                } else {
                    hoodServo.setPosition(intakeMaxRange);
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
                servoFlipper.setPosition(.45);
            }
            if (gamepad1.yWasReleased()) {
                servoFlipper.setPosition(.18);
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
                    hoodServo.setPosition(intakeMinRage);
                    isHoodOpen = false;
                } else {
                    hoodServo.setPosition(intakeMaxRange);
                    isHoodOpen = true;
                }
            }

            telemetry.update();
        }
    }
}

