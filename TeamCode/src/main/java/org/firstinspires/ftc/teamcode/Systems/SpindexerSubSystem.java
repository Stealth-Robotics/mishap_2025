package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.Arrays;

@Configurable
public class SpindexerSubsystem {
    private final DcMotorEx spindexer;
    private int lastTargetPosition; // Variable to store the position before floating

    // --- Constants for your GoBilda 312 RPM Motor ---
    // From the GoBilda website, the 19.2:1 ratio motor has 537.7 Ticks per Revolution.
    public static final double TICKS_PER_REV = 537.7;
    public static final int NUMBER_OF_SLOTS = 3; // You have 3 slots for 3 balls

    // --- Calculated Position Constants ---
    // The number of encoder ticks needed to move one slot.
    public static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    // You can adjust this to control the speed of the spindexer rotation.
    public static final double SPINDEXER_POWER_LIMIT = .17;

    // --- PIDF Tuning ---
    // These coefficients are used for RUN_TO_POSITION mode.
    // P (Proportional): Increases holding power. Fights stiction. (SDK default: 10.0)
    // I (Integral): Corrects for steady-state error. Helps hold against gravity. (SDK default: 3.0)
    // D (Derivative): Dampens overshoot and oscillation. (SDK default: 0.0)
    // F (Friction): Provides an overriding power of friction.
    public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(18, 6, 0, 4);

    // The tolerance for when the shooter is ready
    public static final int POSITION_TOLERANCE = 5;

    public SlotState[] slotStates = new SlotState[NUMBER_OF_SLOTS];

    // FIX: Initialized to a valid starting slot, 0.
    private int currentSlot = 0;

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");

        // It's good practice to set a direction. You might need to change this to FORWARD.
        spindexer.setDirection(DcMotorEx.Direction.REVERSE);

        // Stop and reset the encoder to a known state on initialization.
        // This makes the current position '0' when the robot starts.
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set a tolerance for how close to the target is "close enough" (in encoder ticks)
        // This can help prevent oscillations around the target.
        spindexer.setTargetPositionTolerance(2);

        // Default to BRAKE mode for holding position.
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer.setTargetPosition(0); // Important to set a target after mode change
        this.lastTargetPosition = 0; // Initialize the stored position
        spindexer.setPower(SPINDEXER_POWER_LIMIT);

        // ...// Apply the defined PIDF coefficients to the motor for RUN_TO_POSITION mode
        spindexer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SPINDEXER_PIDF);

        // Set the motor to use the RUN_TO_POSITION mode.
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.rotateToSlot(0);
        this.setSlotsEmpty();
    }

    public void setSlotsEmpty() {
        Arrays.fill(slotStates, SlotState.EMPTY);
    }

    public void setSlotsAuto(){
        Arrays.fill(slotStates, SlotState.ARTIFACT_PURPLE);
        slotStates[0] = SlotState.ARTIFACT_GREEN;
        currentSlot = 0;
    }

    public void setSlotState(int slotNumber, SlotState state) {
        int wrappedSlotNumber = slotNumber % NUMBER_OF_SLOTS;
        slotStates[wrappedSlotNumber] = state;
    }

    public SlotState getSlotState(int slotNumber) {
        int wrappedSlotNumber = slotNumber % NUMBER_OF_SLOTS;
        return slotStates[wrappedSlotNumber];
    }

    public int getCurrentSlot() {
        return currentSlot;
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
        this.lastTargetPosition = targetPosition; // Store the new target

        // Ensure we are in the correct mode for position control
        if (spindexer.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        spindexer.setTargetPosition(targetPosition);
        spindexer.setPower(SPINDEXER_POWER_LIMIT);
        currentSlot = slotNumber;
    }

    /**
     * Advances the spindexer by one slot from its current target position.
     */
    public void advanceOneSlot() {
        // Ensure we are in the correct mode for position control
        if (spindexer.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // We use lastTargetPosition as the authoritative source for the next move
        int newTarget = this.lastTargetPosition + (int)Math.round(TICKS_PER_SLOT);
        this.lastTargetPosition = newTarget; // Store the new target

        spindexer.setTargetPosition(newTarget);
        spindexer.setPower(SPINDEXER_POWER_LIMIT);
        currentSlot = (currentSlot + 1) % NUMBER_OF_SLOTS;
    }

    /**
     * Moves the spindexer back by one slot from its current target position.
     */
    public void decreaseOneSlot() {
        // Ensure we are in the correct mode for position control
        if (spindexer.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // We use lastTargetPosition as the authoritative source for the next move
        int newTarget = this.lastTargetPosition - (int)Math.round(TICKS_PER_SLOT);
        this.lastTargetPosition = newTarget; // Store the new target

        spindexer.setTargetPosition(newTarget);
        spindexer.setPower(SPINDEXER_POWER_LIMIT);

        // FIX: This correctly handles wrapping for negative numbers.
        // (0 - 1 + 3) % 3 = 2.
        // (2 - 1 + 3) % 3 = 1.
        currentSlot = (currentSlot - 1 + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
    }

    /**
     * Adjusts the spindexer's target position by a small amount for fine-tuning.
     * @param ticksToNudge The number of encoder ticks to move the target by.
     *                     Positive values move forward, negative values move backward.
     */
    public void nudgePosition(int ticksToNudge) {
        // Calculate the new target by adjusting the last known target position.
        int newTarget = this.lastTargetPosition + ticksToNudge;
        this.lastTargetPosition = newTarget; // Store the newly nudged target

        // Ensure we are in a state to execute a position command.
        if (spindexer.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        // Command the motor to move to the new, fine-tuned position.
        spindexer.setTargetPosition(newTarget);
        spindexer.setPower(SPINDEXER_POWER_LIMIT);
    }

    /**
     * Puts the motor into a state where it can spin freely, while still tracking position.
     * This is ideal for allowing game elements to turn the spindexer.
     */
    public void setFloat() {
        // Disables brake mode
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Switch to a mode that doesn't actively hold position
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Ensure motor power is zero to allow floating
        spindexer.setPower(0);
    }

    /**
     * Puts the motor into a state where it actively holds its position.
     * It will return to the position it was targeting before setFloat() was called.
     */
    public void setBrake() {
        // Set the target back to the last known target position before float was called.
        spindexer.setTargetPosition(this.lastTargetPosition);
        // Switch back to position-holding mode.
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Apply power so the motor can move to and hold the target.
        // The PID controller will reduce this to a holding power automatically.
        spindexer.setPower(SPINDEXER_POWER_LIMIT);
    }

    /**
     * Checks if the motor is currently busy moving to a target position.
     * @return False if the motor is still moving, true if it has reached its target.
     */
    public boolean isReady() {
        int error = Math.abs(spindexer.getTargetPosition() - spindexer.getCurrentPosition());

        // Return true if the error is within our acceptable tolerance.
        return error <= POSITION_TOLERANCE;
    }

    /**
     * Stops the spindexer motor.
     */
    public void stop() {
        spindexer.setPower(0);
    }

    /**
     * Allows for manual spinning of the spindexer.
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
        spindexer.setTargetPosition(0);
        this.lastTargetPosition = 0; // Also reset the stored target
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // After resetting encoders, our logical slot should also be zero.
        this.currentSlot = 0;
    }

    /**
     * Gets the current encoder position of the motor.
     * @return The current position in encoder ticks.
     */
    public double getPosition() {
        return spindexer.getCurrentPosition();
    }

    public enum SlotState {
        EMPTY,
        ARTIFACT_PURPLE,
        ARTIFACT_GREEN,
        UNKNOWN
    }
}
