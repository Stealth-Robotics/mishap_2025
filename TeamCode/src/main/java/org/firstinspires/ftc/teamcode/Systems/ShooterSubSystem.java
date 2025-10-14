package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class ShooterSubSystem {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;

    // Constants for motor RPM. Your GoBILDA motor has a 28 PPR encoder.
    public static final double MAX_RPM = 5000;
    public static final double MIN_RPM = 1000;
    public static final double DEFAULT_RPM = 4000;
    public static final double RPM_CHANGE_AMOUNT = 50;
    public static final double VELOCITY_TOLERANCE = 25; // The allowed RPM error

    // These are the ticks per revolution for a standard GoBILDA Yellow Jacket motor.
    public static final double TICKS_PER_REV = 28;

    // TODO: You MUST tune these PIDF coefficients for your specific shooter setup for optimal performance.
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(50, 0, 10, 13);

    public double targetRPM = DEFAULT_RPM;

    public ShooterSubSystem(HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, "right_shoot_motor");
        leftShooter = hardwareMap.get(DcMotorEx.class, "left_shoot_motor");

        // Set motor directions if one is mounted opposite to the other
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motors to run using encoders for velocity control
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set PIDF coefficients for velocity control
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
    }

    /**
     * Sets the shooter motors to a specific RPM.
     * @param rpm The target revolutions per minute.
     */
    public void shoot(double rpm) {
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }
        targetRPM = rpm;

        // Convert RPM to ticks per second for the setVelocity() method
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        rightShooter.setVelocity(ticksPerSecond);
        leftShooter.setVelocity(ticksPerSecond);
    }

    /**
     * Runs the shooter at the current targetRPM.
     */
    public void shoot() {
        shoot(targetRPM);
    }

    /**
     * Stops the shooter motors.
     */
    public void stop(){
        rightShooter.setPower(0);
        leftShooter.setPower(0);
    }

    /**
     * Increases the target RPM.
     */
    public void increaseRPM() {
        if (targetRPM >= MAX_RPM) {
            return;
        }
        targetRPM += RPM_CHANGE_AMOUNT;
    }

    /**
     * Decreases the target RPM.
     */
    public void decreaseRPM() {
        if (targetRPM <= MIN_RPM) {
            return;
        }
        targetRPM -= RPM_CHANGE_AMOUNT;
    }

    /**
     * Checks if the shooter motors are at their target RPM within a tolerance.
     * @return True if the motors are at the target speed.
     */
    public boolean shootReady() {
        double targetVelocity = targetRPM * TICKS_PER_REV / 60;
        double currentRightVelocity = rightShooter.getVelocity();
        double currentLeftVelocity = leftShooter.getVelocity();

        double rightError = Math.abs(targetVelocity - currentRightVelocity);
        double leftError = Math.abs(targetVelocity - currentLeftVelocity);

        return (rightError <= VELOCITY_TOLERANCE) && (leftError <= VELOCITY_TOLERANCE);
    }

    /**
     * Gets the current average RPM of the shooter motors.
     * @return The average RPM.
     */
    public double getCurrentRPM() {
        // Convert the average ticks per second back to RPM
        double avgTicksPerSecond = (rightShooter.getVelocity() + leftShooter.getVelocity()) / 2;
        return (avgTicksPerSecond / TICKS_PER_REV) * 60;
    }

    /**
     * Gets the target RPM.
     * @return The target RPM.
     */
    public double getTargetRPM() {
        return targetRPM;
    }
}
