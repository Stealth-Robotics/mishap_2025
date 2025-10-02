package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Runner", group="Tests")
public class ServoRunner extends LinearOpMode {
    private boolean isIntake = false;
    private boolean isOuttake = false;

    private boolean isHoodOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo hopperServo = hardwareMap.get(Servo.class, "hopperServo");
        CRServo servoIntakeRight = hardwareMap.get(CRServo.class, "servoIntakeRight");
        CRServo servoIntakeLeft = hardwareMap.get(CRServo.class, "servoIntakeLeft");
        Servo servoFlipper = hardwareMap.get(Servo.class, "servoFlipper");

        servoIntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        telemetry.addLine("Started");
        telemetry.update();
        servoFlipper.setPosition(.2);
        while (opModeIsActive()) {
            telemetry.addLine("Running");
            telemetry.update();
            telemetry.addData("hopperServo position:", hopperServo.getPosition());
            //hopperServo.setPosition(gamepad1.right_trigger * 1);
            telemetry.addData("hopperServo position:", hopperServo.getPosition());

            if (gamepad1.rightBumperWasPressed()) {
                if (isIntake) {
                    isIntake = false;
                    servoIntakeRight.setPower(0);
                    servoIntakeLeft.setPower(0);
                } else {
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

            if (gamepad1.aWasPressed()) {
                if (isHoodOpen) {
                    hopperServo.setPosition(0);
                    isHoodOpen = false;
                } else {
                    hopperServo.setPosition(0.22);
                    isHoodOpen = true;
                }
            }
        }
    }
}

