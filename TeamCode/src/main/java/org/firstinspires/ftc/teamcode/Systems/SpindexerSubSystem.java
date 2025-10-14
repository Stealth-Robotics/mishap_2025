package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SpindexerSubSystem {
    private final DcMotorEx spindexer;

    // --- Constants for your GoBilda 320 RPM Motor ---
    // From the GoBilda website, the 19.2:1 ratio motor has 537.7 Ticks per Revolution.
    public static final double TICKS_PER_REV = 537.7;
    public static final int NUMBER_OF_SLOTS = 3; // You have 3 slots for 3 balls

    // --- Calculated Position Constants ---
    // The number of encoder ticks needed to move one slot.
    public static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    // You can adjust this to control the speed of the spindexer rotation.
    public static final double SPINDEXER_POWER_LIMIT = 0.7;


    public SpindexerSubSystem(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");

        // It's good practice to set a direction. You might need to change this to FORWARD.
        spindexer.setDirection(DcMotorEx.Direction.FORWARD);

        // Stop and reset the encoder to a known state on initialization.
        // This makes the current position '0' when the robot starts.
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the motor to use the RUN_TO_POSITION mode.
        // This enables PID control for precise positioning.
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set a tolerance for how close to the target is "close enough" (in encoder ticks)
        // This can help prevent oscillations around the target.
        spindexer.setTargetPositionTolerance(10);
        resetEncoder();
    }

    /**
     * Rotates the spindexer to a specific slot number.
     * @param slotNumber The slot to rotate to (e.g., 0, 1, 2).
     */
    public void rotateToSlot(int slotNumber) {
        // Ensure the slot number is valid to prevent errors
        if (slotNumber < 0 || slotNumber >= NUMBER_OF_SLOTS) {
            return; // Or handle the error appropriately
        }

        // Calculate the target position in encoder ticks
        int targetPosition = (int) Math.round(slotNumber * TICKS_PER_SLOT);

        spindexer.setTargetPosition(targetPosition);
        spindexer.setPower(SPINDEXER_POWER_LIMIT);
    }

    /**
     * Advances the spindexer by one slot from its current target position.
     */
    public void advanceOneSlot() {
        int currentTarget = spindexer.getTargetPosition();
        int newTarget = currentTarget + (int)Math.round(TICKS_PER_SLOT);

        spindexer.setTargetPosition(newTarget);
        spindexer.setPower(SPINDEXER_POWER_LIMIT);
    }

    public void decreaseOneSlot() {
        int currentTarget = spindexer.getTargetPosition();
        int newTarget = currentTarget - (int)Math.round(TICKS_PER_SLOT);

        spindexer.setTargetPosition(newTarget);
    }

    /**
     * Checks if the motor is currently busy moving to a target position.
     * @return True if the motor is still moving, false if it has reached its target.
     */
    public boolean isBusy() {
        return spindexer.isBusy();
    }

    /**
     * Stops the spindexer motor. Note: In RUN_TO_POSITION mode, the motor
     * will automatically stop when it reaches the target. This is for emergency stops.
     */
    public void stop() {
        // Setting power to 0 in RUN_TO_POSITION will stop the motor from moving
        // towards its target, but it will still hold its position if disturbed.
        spindexer.setPower(0);
    }

    /**
     * Allows for manual spinning of the spindexer. This requires changing the run mode.
     * Useful for testing or manual overrides.
     * @param power The power to apply to the motor (-1.0 to 1.0).
     */
    public void manualSpin(double power) {
        // We must change the mode to run without encoders to use setPower for continuous rotation.
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(power);
    }

    /**
     * Resets the current position of the spindexer to be the new zero point.
     * Call this when the spindexer is in its known "home" or starting position.
     */
    public void resetEncoder() {
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // It's crucial to set the mode back to RUN_TO_POSITION after resetting.
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Gets the current encoder position of the motor.
     * @return The current position in encoder ticks.
     */
    public double getPosition() {
        return spindexer.getCurrentPosition();
    }
}
