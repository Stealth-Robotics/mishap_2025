package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.common.MotorVelocityReader;

/**
 * Manages the dual-motor shooter mechanism of the robot.
 * This subsystem handles spinning the motors to a target velocity (RPM),
 * adjusting target speeds, and checking if the shooter is ready to fire.
 */
public class ShooterSubsystem {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;

    private final MotorVelocityReader rightVelocityReader;
    private final MotorVelocityReader leftVelocityReader;


    // --- Constants ---
    public static final double MAX_RPM = 5200;
    public static final double MIN_RPM = 0;
    public static final double DEFAULT_RPM_FAR = 4800;
    public static final double DEFAULT_RPM_NEAR = 4200;
    public static final double RPM_CHANGE_AMOUNT = 100;
    private static final double VELOCITY_TOLERANCE = 50; // The allowed RPM error in which the shooter is considered "ready".

    // Encoder ticks per revolution for a GoBILDA Yellow Jacket motor.
    private static final double TICKS_PER_REV = 28;

    // IMPORTANT: These PIDF coefficients MUST be tuned for your specific shooter setup for optimal performance.
    // They control how the motor corrects its speed to match the target.
    // P (Proportional): Reduces large errors.
    // I (Integral): Corrects for small, steady-state errors.
    // D (Derivative): Prevents overshooting the target.
    // F (Feedforward): Proactively applies power based on the target velocity, which is crucial for velocity control.
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(90, .5, 10, 13);

    // --- State Variables ---
    private double targetRpmFar = DEFAULT_RPM_FAR;
    private double targetRpmNear = DEFAULT_RPM_NEAR;
    private boolean isNear = false;
    private double currentRpm = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        rightShooter = hardwareMap.get(DcMotorEx.class, "right_shoot_motor");
        leftShooter = hardwareMap.get(DcMotorEx.class, "left_shoot_motor");

        // Reverse one motor if they are mounted in opposition, so they spin in the same direction.
        rightShooter.setDirection(DcMotorEx.Direction.REVERSE);

        // Set motors to run using encoders for velocity control. This is essential for RPM-based shooting.
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Apply the tuned PIDF coefficients for velocity control.
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);

        // Initialize velocity readers to get filtered RPM values from the motors.
        rightVelocityReader = new MotorVelocityReader(rightShooter, TICKS_PER_REV);
        leftVelocityReader = new MotorVelocityReader(leftShooter, TICKS_PER_REV);
    }

    /**
     * This method should be called in the main robot loop to update the shooter's state,
     * such as the current average RPM.
     */
    public void update(){
        updateAverageRpm();
    }

    /**<caret>
     * Sets the internal target RPM for both shooter motors.
     * This method includes safety checks to ensure the RPM is within the valid range.
     * @param rpm The target revolutions per minute.
     */
    private void setRpms(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }

        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        rightShooter.setVelocity(ticksPerSecond);
        leftShooter.setVelocity(ticksPerSecond);
    }

    /**
     * Runs the shooter motors at the currently selected target RPM (near or far).
     */
    public void runShooter() {
        setRpms(isNear ? targetRpmNear : targetRpmFar);
    }

    /**
     * Stops the shooter motors.
     */
    public void stop(){
        setRpms(0); // Setting RPM to 0 is the correct way to stop motors in velocity control mode.
    }

    /**
     * Increases the target RPM for the 'near' shot distance.
     * The change only applies immediately if the shooter is already running.
     */
    public void increaseRpmNear() {
        if(targetRpmNear >= MAX_RPM){
            return;
        }

        targetRpmNear += RPM_CHANGE_AMOUNT;
        // If the shooter is already running and set to near, update the speed instantly.
        if(isRunning() && isNear) {
            setRpms(targetRpmNear);
        }
    }

    /**
     * Decreases the target RPM for the 'near' shot distance.
     */
    public void decreaseRpmNear() {
        if(targetRpmNear <= MIN_RPM){
            return;
        }

        targetRpmNear -= RPM_CHANGE_AMOUNT;
        if(isRunning() && isNear) {
            setRpms(targetRpmNear);
        }
    }

    /**
     * Increases the target RPM for the 'far' shot distance.
     */
    public void increaseRpmFar() {
        if (targetRpmFar >= MAX_RPM) {
            return;
        }

        targetRpmFar += RPM_CHANGE_AMOUNT;
        // If the shooter is already running and set to far, update the speed instantly.
        if (isRunning() && !isNear) {
            setRpms(targetRpmFar);
        }
    }

    /**
     * Decreases the target RPM for the 'far' shot distance.
     */
    public void decreaseRpmFar() {
        if (targetRpmFar <= MIN_RPM) {
            return;
        }

        targetRpmFar -= RPM_CHANGE_AMOUNT;
        if (isRunning() && !isNear) {
            setRpms(targetRpmFar);
        }
    }

    /**
     * Gets the currently selected target RPM (either near or far).
     * @return The active target RPM.
     */
    public double getTargetRpm(){
        return isNear ? targetRpmNear : targetRpmFar;
    }

    /**
     * Checks if the shooter motors are at their target RPM within a defined tolerance.
     * This is useful for determining when it's safe to feed a projectile.
     * @return True if the motors are at the target speed.
     */
    public boolean isReadyToShoot() {
        // Ensure the shooter is actually supposed to be running before checking if it's "ready".
        if (!isRunning()) {
            return false;
        }

        double error = Math.abs(currentRpm - getTargetRpm());
        return error <= VELOCITY_TOLERANCE;
    }

    /**
     * Checks if the shooter motors are currently running.
     * @return True if the motors have a velocity greater than a minimum threshold.
     */
    public boolean isRunning() {
        // A motor's velocity might not be exactly 0 due to physics, so check against a small threshold.
        return rightShooter.getVelocity() > 1 || leftShooter.getVelocity() > 1;
    }

    /**
     * Gets the current average RPM of the shooter motors.
     * @return The average RPM as calculated by the velocity readers.
     */
    public double getCurrentRpm() {
        return currentRpm;
    }

    /**
     * Gets the configured target RPM for far shots.
     * @return The target RPM for the far distance.
     */
    public double getTargetRpmFar() {
        return targetRpmFar;
    }

    /**
     * Gets the configured target RPM for near shots.
     * @return The target RPM for the near distance.
     */
    public double getTargetRpmNear() {
        return targetRpmNear;
    }

    /**
     * Sets the target shooting range.
     * @param isNear True to select the near target RPM, false for the far target RPM.
     */
    public void setTargetRange(boolean isNear) {
        this.isNear = isNear;
    }

    /**
     * Updates the current average RPM from the two motor velocity readers.
     * This should be called periodically in the main loop via update().
     */
    private void updateAverageRpm() {
        this.currentRpm = (rightVelocityReader.getFilteredRpm() + leftVelocityReader.getFilteredRpm()) / 2.0;
    }
}
