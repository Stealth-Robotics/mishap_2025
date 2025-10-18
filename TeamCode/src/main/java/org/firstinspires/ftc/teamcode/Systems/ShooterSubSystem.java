package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Common.MotorVelocityReader;

public class ShooterSubsystem {
    private final DcMotorEx rightShooter;
    private final DcMotorEx leftShooter;

    private final MotorVelocityReader rightVelocityReader;
    private final MotorVelocityReader leftVelocityReader;


    // Constants for motor RPM. Your GoBILDA motor has a 28 PPR encoder.
    public static final double MAX_RPM = 5100;
    public static final double MIN_RPM = 0;
    public static final double DEFAULT_RPM_FAR = 4500;
    public static final double DEFAULT_RPM_NEAR = 3000;
    public static final double RPM_CHANGE_AMOUNT = 100;
    private static final double VELOCITY_TOLERANCE = 50; // The allowed RPM error

    // These are the ticks per revolution for a standard GoBILDA Yellow Jacket motor.
    private static final double TICKS_PER_REV = 28;

    // TODO: You MUST tune these PIDF coefficients for your specific shooter setup for optimal performance.
    private static final PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(90, .5, 10, 13);

    private double targetRpmFar = DEFAULT_RPM_FAR;

    private double targetRpmNear = DEFAULT_RPM_NEAR;

    private Boolean isNear = false;
    private double currentRpm = 0;


    public ShooterSubsystem(HardwareMap hardwareMap) {
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
        // Initialize the new velocity readers
        rightVelocityReader = new MotorVelocityReader(rightShooter, TICKS_PER_REV);
        leftVelocityReader = new MotorVelocityReader(leftShooter, TICKS_PER_REV);
    }

    public void update(){
        updateAverageRpm();
    }

    /**
     * Sets the shooter motors to a specific RPM.
     * @param rpm The target revolutions per minute.
     */
    private void setRpms(double rpm) {
        if (rpm > MAX_RPM) {
            rpm = MAX_RPM;
        } else if (rpm < MIN_RPM) {
            rpm = MIN_RPM;
        }

        // Convert RPM to ticks per second for the setVelocity() method
        double ticksPerSecond = rpm * TICKS_PER_REV / 60;
        rightShooter.setVelocity(ticksPerSecond);
        leftShooter.setVelocity(ticksPerSecond);
    }

    /**
     * Runs the shooter at the current targetRPM.
     */
        public void spinUpShooter() {
            setRpms(isNear ? targetRpmNear : targetRpmFar);
    }

    /**
     * Stops the shooter motors.
     */
    public void stop(){
        rightShooter.setVelocity(0);
        leftShooter.setVelocity(0);
    }

    /**
     * Increases the target RPM.
     */
    public void increaseRPMNear() {
        if(targetRpmNear >= MAX_RPM){
            return;
        }

        targetRpmNear += RPM_CHANGE_AMOUNT;
        if(getCurrentRpm() > MIN_RPM) {
            setRpms(targetRpmNear);
        }
    }

    /**
     * Decreases the target RPM.
     */
    public void decreaseRPMNear() {
        if(targetRpmNear <= MIN_RPM){
            return;
        }

        targetRpmNear -= RPM_CHANGE_AMOUNT;
        if(getCurrentRpm() > MIN_RPM) {
            setRpms(targetRpmNear);
        }
    }

    /**
     * Increases the target RPM.
     */
    public void increaseRpmFar() {
        if (targetRpmFar >= MAX_RPM) {
            return;
        }

        targetRpmFar += RPM_CHANGE_AMOUNT;
        // If the motors a running increase speed now
        if (getCurrentRpm() > MIN_RPM) {
            setRpms(targetRpmFar);
        }
    }

    /**
     * Decreases the target RPM.
     */
    public void decreaseRpmFar() {
        if (targetRpmFar <= MIN_RPM) {
            return;
        }

        targetRpmFar -= RPM_CHANGE_AMOUNT;
        if (getCurrentRpm() > MIN_RPM) {
            setRpms(targetRpmFar);
        }
    }

    public double getTargetRpm(){
        return isNear ? targetRpmNear : targetRpmFar;
    }

    public void SpinDown() {
        setRpms(0);
    }

    /**
     * Checks if the shooter motors are at their target RPM within a tolerance.
     * @return True if the motors are at the target speed.
     */
    public boolean isShootReady() {
        double currentRPM = getCurrentRpm();
        double targetRPM = isNear ? targetRpmNear : targetRpmFar;
        double error = Math.abs(currentRPM - targetRPM);
        return error <= VELOCITY_TOLERANCE;
    }

    /**
     * Gets the current average RPM of the shooter motors.
     * @return The average RPM.
     */
    public double getCurrentRpm() {
        return currentRpm;
    }

    /**
     * Gets the target RPM.
     * @return The target RPM.
     */
    public double getTargetRpmFar() {
        return targetRpmFar;
    }

    public double getTargetRpmNear() {
        return targetRpmNear;
    }

    public void setTargetRange(boolean isNear) {
        this.isNear = isNear;
    }

    private void updateAverageRpm() {
        this.currentRpm = (rightVelocityReader.getRpm() + leftVelocityReader.getRpm()) / 2;
    }

}
