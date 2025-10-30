package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.MotorVelocityReader;

import java.util.HashMap;
import java.util.Map;

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

    private final ElapsedTime shootTimer = new ElapsedTime();

    private static final double SHOOT_RANGE_NEAR = 50;
    private static final double SHOOT_RANGE_FAR = 80;

    private static final double MIN_SHOOT_TIME_MS = 400;
    private static final double MAX_SHOOT_TIME_MS = 3000;

    // --- Constants ---
    public static final double MAX_RPM = 5200;
    public static final double MIN_RPM = 3000;
    public static final double DEFAULT_RPM_FAR = 4800;
    public static final double DEFAULT_RPM_NEAR = 4200;
    public static final double DEFFAULT_RPM_MID = 4400;
    public static final double RPM_CHANGE_AMOUNT = 50;
    private static final double VELOCITY_TOLERANCE = 60; // The allowed RPM error in which the shooter is considered "ready".

    // Encoder ticks per revolution for a GoBILDA Yellow Jacket motor.
    private static final double TICKS_PER_REV = 28;

    // IMPORTANT: These PIDF coefficients MUST be tuned for your specific shooter setup for optimal performance.
    // They control how the motor corrects its speed to match the target.
    // P (Proportional): Reduces large errors.
    // I (Integral): Corrects for small, steady-state errors.
    // D (Derivative): Prevents overshooting the target.
    // F (Feedforward): Proactively applies power based on the target velocity, which is crucial for velocity control.
    // TODO: more tuning needed
 //   private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(90, .5, 10, 13);
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(1.3, 0.15, 0, 12.1);

    // --- State Variables ---
    private boolean isShooterEnabled = false; // New state to track if the shooter is supposed to be running
    private double currentRpm = 0;
    private boolean isNear = false; // Restored this state variable

    private RpmRange currentRpmRange = RpmRange.FAR;

    private enum RpmRange{
        FAR,
        MID,
        NEAR
    }

    // Holds the different set point for the shooter
    private final Map<RpmRange, Double> rangeMap;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        rangeMap = new HashMap<>();
        rangeMap.put(RpmRange.FAR, DEFAULT_RPM_FAR);
        rangeMap.put(RpmRange.MID, DEFFAULT_RPM_MID);
        rangeMap.put(RpmRange.NEAR, DEFAULT_RPM_NEAR);

        // Initialize the shooter motors from the hardware map.
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
     * Sets the target RPM for the shooter based on the current shooting range.
     * @param distanceInch distance in inches to target
     * TODO: use a range Pid to control the target rpm based on distance
     */
    public void setTargetRpmFromDisance(double distanceInch){

        if (distanceInch < SHOOT_RANGE_NEAR){
            currentRpmRange = RpmRange.NEAR;
        }else if (distanceInch < SHOOT_RANGE_FAR) {
            currentRpmRange = RpmRange.MID;
        }else {
            currentRpmRange = RpmRange.FAR;
        }
    }
    /**
     * This method should be called in the main robot loop to update the shooter's state,
     * such as the current average RPM.
     */
    public void update(){
        updateAverageRpm();
    }

    /**
     * Sets the internal target RPM for both shooter motors.
     * This method includes safety checks to ensure the RPM is within the valid range.
     * @param rpm The target revolutions per minute.
     */
    private void setRpm(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm > 0 && rpm < MIN_RPM) {
            // If setting a non-zero RPM, ensure it meets the minimum operational speed
            rpm = MIN_RPM;
        }


        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        rightShooter.setVelocity(ticksPerSecond);
        leftShooter.setVelocity(ticksPerSecond);
        resetVelocityReaders();
    }

    /**
     * Runs the shooter motors at the currently selected target RPM (near or far).
     */
    public void runShooter() {
        isShooterEnabled = true; // Set the intended state to ON
        shootTimer.reset();

        //noinspection DataFlowIssue
        setRpm(rangeMap.get(currentRpmRange));
    }

    /**
     * Stops the shooter motors.
     */
    public void stop(){
        isShooterEnabled = false; // Set the intended state to OFF
        setRpm(0); // Setting RPM to 0 is the correct way to stop motors in velocity control mode.
        shootTimer.reset();
    }

    /**
     * Increases the target RPM for the 'far' shot distance.
     */
    public void increaseCurrentRpmRange() {
        //noinspection DataFlowIssue
        double curRpm = rangeMap.get(currentRpmRange);

        if (curRpm >= MAX_RPM) {
            return;
        }

        curRpm += RPM_CHANGE_AMOUNT;
        rangeMap.put(currentRpmRange, curRpm);
        if (isShooterEnabled){
            setRpm(curRpm);
        }

    }

    public void resetRpmCounter() {
        this.rightVelocityReader.reset();
        this.leftVelocityReader.reset();
    }

    /**
     * Decreases the target RPM for the 'far' shot distance.
     */
    public void decreaseCurrentRpmRange() {
        //noinspection DataFlowIssue
        double curRpm = rangeMap.get(currentRpmRange);

        if (curRpm <= MIN_RPM) {
            return;
        }

        curRpm -= RPM_CHANGE_AMOUNT;
        rangeMap.put(currentRpmRange, curRpm);

        if (isShooterEnabled) {
            setRpm(curRpm);
        }
    }

    /**
     * Gets the currently selected target RPM (either near or far).
     * @return The active target RPM.
     */
    public double getTargetRpm(){
        //noinspection DataFlowIssue
        return rangeMap.get(currentRpmRange);
    }

    /**
     * Checks if the shooter motors are at their target RPM within a defined tolerance.
     * This is useful for determining when it's safe to feed a projectile.
     * @return True if the motors are at the target speed.
     */
    public boolean isReadyToShoot() {
        // Ensure the shooter is actually supposed to be running before checking if it's "ready".
        if (!isShooterEnabled) { // Use the new state variable for the primary check
            return false;
        }

        // Check if the minimum shoot time has elapsed.
        double curMs = shootTimer.milliseconds();
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }

        // The current RPM is updated periodically by the update() method
        double error = Math.abs(currentRpm - getTargetRpm());

        // if for some reason the shooter cant reach requested RPM just shoot it
        return error <= VELOCITY_TOLERANCE || curMs > MAX_SHOOT_TIME_MS;
    }

    /**
     * Checks if the shooter motors are currently running (physically spinning).
     * @return True if the motors have a velocity greater than a minimum threshold.
     */
    public boolean isRunning() {
        // This method checks the actual motor velocity. It can differ from isShooterEnabled during spin-up/down.
        return rightShooter.getVelocity() > 100 && leftShooter.getVelocity() > 100;
    }

    /**
     * Gets the current average RPM of the shooter motors.
     * @return The average RPM as calculated by the velocity readers.
     */
    public double getCurrentRpm() {
        return currentRpm;
    }

    /**
     * Sets the target shooting range.
     * @param isNear True to select the near target RPM, false for the far target RPM.
     */
    public void setTargetRange(boolean isNear) {
        this.isNear = isNear;
        // Update the currentRpmRange based on the new isNear value
        if (isNear) {
            this.currentRpmRange = RpmRange.NEAR;
        } else {
            // You might want to decide if this defaults to FAR or MID. I'll assume FAR.
            this.currentRpmRange = RpmRange.FAR;
        }

        // If the shooter is already running, update its speed to the new target
        if (isShooterEnabled) {
            runShooter();
        }
    }

    private void resetVelocityReaders(){
        leftVelocityReader.reset();
        rightVelocityReader.reset();
        shootTimer.reset();
    }

    public double getMotorRpms() {
        return (((this.rightShooter.getVelocity() + leftShooter.getVelocity()) / 2.0) / TICKS_PER_REV) * 60;
    }


    /**
     * Updates the current average RPM from the two motor velocity readers.
     * This should be called periodically in the main loop via update().
     */
    private void updateAverageRpm() {
        // Use the absolute value to prevent issues with one motor reporting a negative value momentarily
        double rightRpm = rightVelocityReader.getFilteredRpm();
        double leftRpm = leftVelocityReader.getFilteredRpm();
        this.currentRpm = (Math.abs(rightRpm) + Math.abs(leftRpm)) / 2.0;
    }
}
