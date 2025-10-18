package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * For calculating TRUE RPM of a motor
 */
public class MotorVelocityReader {
    private final DcMotorEx motor;
    private final double ticksPerRev;
    private final ElapsedTime timer = new ElapsedTime();
    private int lastPosition;
    private double lastTime;

    private double lastRpm = 0;

    public MotorVelocityReader(DcMotorEx motor, double ticksPerRev) {
        this.motor = motor;
        this.ticksPerRev = ticksPerRev;
        reset();
    }

    /**
     * Resets the baseline for velocity calculation. This should be called once
     * when you are ready to start measuring, for example, at the beginning of runOpMode.
     */
    public void reset() {
        lastPosition = motor.getCurrentPosition();
        lastTime = timer.seconds();
        lastRpm = 0;
    }

    /**
     * Calculates the motor's current RPM based on encoder ticks and time elapsed.
     * This should be called periodically in a loop.
     * @return The current, measured RPM of the motor.
     */
    public double getRpm() {
        double currentTime = timer.seconds();
        int currentPosition = motor.getCurrentPosition();

        double deltaTime = currentTime - lastTime;

        // If not enough time has passed, return the previously calculated RPM.
        // Also protects against division by zero.
        if (deltaTime < 0.02) { // Use a small interval, not 1 second.
            return lastRpm;
        }

        int deltaTicks = currentPosition - lastPosition;
        double ticksPerSecond = deltaTicks / deltaTime;
        double revolutionsPerSecond = ticksPerSecond / ticksPerRev;

        double currentRpm = revolutionsPerSecond * 60;

        // Update state for the next calculation
        lastTime = currentTime;
        lastPosition = currentPosition;
        lastRpm = currentRpm; // Store the newly calculated RPM

        return currentRpm;
    }
}
