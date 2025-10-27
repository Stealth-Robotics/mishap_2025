package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.MotorVelocityReader;

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

    private final ElapsedTime minShootTimer = new ElapsedTime();

    private static final double SHOOT_RANGE_NEAR = 60;
    private static final double SHOOT_RANGE_FAR = 90;

    private static final double MIN_SHOOT_TIME_MS = 500;
    private static final double MAX_SHOOT_TIME_MS = 3000;

    // --- Constants ---
    public static final double MAX_RPM = 5200;
    public static final double MIN_RPM = 0;
    public static final double DEFAULT_RPM_FAR = 4850;
    public static final double DEFAULT_RPM_NEAR = 4200;

    public static final double DEFFAULT_RPM_MID = 4400;
    public static final double RPM_CHANGE_AMOUNT = 100;
    private static final double VELOCITY_TOLERANCE = 500; // The allowed RPM error in which the shooter is considered "ready".

    // Encoder ticks per revolution for a GoBILDA Yellow Jacket motor.
    private static final double TICKS_PER_REV = 28;

    // IMPORTANT: These PIDF coefficients MUST be tuned for your specific shooter setup for optimal performance.
    // They control how the motor corrects its speed to match the target.
    // P (Proportional): Reduces large errors.
    // I (Integral): Corrects for small, steady-state errors.
    // D (Derivative): Prevents overshooting the target.
    // F (Feedforward): Proactively applies power based on the target velocity, which is crucial for velocity control.
    // TODO: more tuning needed
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(90, .5, 10, 13);

    // --- State Variables ---
    private double targetRpmFar = DEFAULT_RPM_FAR;
    private double targetRpmNear = DEFAULT_RPM_NEAR;
    private double targetRpmMid = DEFFAULT_RPM_MID;
    private boolean isNear = false;
    private double currentRpm = 0;

    private RpmRange currentRpmRange = RpmRange.FAR;

    private enum RpmRange{
        FAR,
        MID,
        NEAR
    }

    private final Map<RpmRange, Double> rangeMap = Map.of(
            RpmRange.NEAR, targetRpmNear,
            RpmRange.MID, targetRpmMid,
            RpmRange.FAR, targetRpmFar
    );


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

    /**<caret>
     * Sets the internal target RPM for both shooter motors.
     * This method includes safety checks to ensure the RPM is within the valid range.
     * @param rpm The target revolutions per minute.
     */
    private void setRpm(double rpm) {
        // Clamp the RPM to the allowable min/max range to prevent motor damage or unexpected behavior.
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }

        currentRpm = rpm;

        // Convert desired RPM to encoder ticks per second, which is the unit required by DcMotorEx.setVelocity().
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        rightShooter.setVelocity(ticksPerSecond);
        leftShooter.setVelocity(ticksPerSecond);
    }

    /**
     * Runs the shooter motors at the currently selected target RPM (near or far).
     */
    public void runShooter() {

        //noinspection DataFlowIssue
        setRpm(rangeMap.get(currentRpmRange));
        minShootTimer.reset();
    }

    /**
     * Stops the shooter motors.
     */
    public void stop(){
        setRpm(0); // Setting RPM to 0 is the correct way to stop motors in velocity control mode.
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
        if (isRunning()){
            setRpm(curRpm);
        }

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

        if (isRunning()) {
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
        if (!isRunning()) {
            return false;
        }

        // Check if the minimum shoot time has elapsed.
        double curMs = minShootTimer.milliseconds();
        if (curMs < MIN_SHOOT_TIME_MS) {
            return false;
        }

        double error = Math.abs(currentRpm - getTargetRpm());
        // if for some reason the shooter cant reach requested RPM just shoot it
        if (error <= VELOCITY_TOLERANCE || curMs > MAX_SHOOT_TIME_MS) {
            minShootTimer.reset();
            return true;
        }

        return false;
    }

    /**
     * Checks if the shooter motors are currently running.
     * @return True if the motors have a velocity greater than a minimum threshold.
     */
    public boolean isRunning() {
        // A motor's velocity might not be exactly 0 due to physics, so check against a small threshold.
        return rightShooter.getVelocity() > 100 || leftShooter.getVelocity() > 100;
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
    }

    /**
     * Updates the current average RPM from the two motor velocity readers.
     * This should be called periodically in the main loop via update().
     */
    private void updateAverageRpm() {
        this.currentRpm = (rightVelocityReader.getFilteredRpm() + leftVelocityReader.getFilteredRpm()) / 2.0;
    }
}
