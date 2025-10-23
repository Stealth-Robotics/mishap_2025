package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.Arrays;
/**
 * Manages the spindexer mechanism, which is a rotating drum that holds and positions artifacts for shooting.
 * This subsystem handles homing, rotating to specific slots, and finding artifacts by color.
 * It uses a state machine for its initialization sequence (`doInitPosition`) and provides
 * methods for both manual and automated control.
 */
@Configurable
public class SpindexerSubsystem {

    //==================================================================================================
    //  C O N S T A N T S
    //==================================================================================================

    // NOTE: if you would like to adjust in FTC dashboard mark members as public static (Not final)

    /** Ticks per revolution for the GoBilda 312 RPM motor (537.7) geared up. */
    public static final double TICKS_PER_REV = 3895.9;
    /** The number of slots in the spindexer. */
    public static final int NUMBER_OF_SLOTS = 3;
    /** The number of encoder ticks needed to move one slot. */
    public static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    /** The number of ticks to move backward after the index switch is released to center a slot. */
    public static final int INDEX_OFFSET_TICKS = 80;

    /** The tolerance, in ticks, for considering the motor to have reached its target position. */
    public static final int POSITION_TOLERANCE = 2;

    /** The maximum power limit for spindexer rotation. */
    public static double SPINDEXER_POWER_LIMIT = 1;
    /** The maximum velocity (in ticks/sec) for spindexer rotation in RUN_TO_POSITION mode. */
    public static double SPINDEXER_VELOCITY_LIMIT = 2500;

    /** PIDF coefficients for position control, tunable via FTC-Dashboard. */
    public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(8, 4,1.5, 1);

    /**
     * Defines the possible states of each slot in the spindexer.
     */
    public enum SlotState {
        EMPTY,
        ARTIFACT_PURPLE,
        ARTIFACT_GREEN,
        UNKNOWN
    }

    //==================================================================================================
    //  P R I V A T E   M E M B E R   V A R I A B L E S
    //==================================================================================================

    private final DcMotorEx spindexer;
    private final TouchSensor indexSwitch;

    /** An array to hold the state of each slot (e.g., EMPTY, ARTIFACT_GREEN). */
    private final SlotState[] slotStates = new SlotState[NUMBER_OF_SLOTS];

    /** The current slot number that is aligned with the shooting or intake mechanism. */
    private int currentSlot = 0;

    /** Stores the last commanded target position, used to resume position after floating. */
    private int lastTargetPosition;

    /** The current state of the homing/initialization process. */
    private IndexingState indexingState = IndexingState.START;

    /**
     * State machine enum to manage the multi-step homing process.
     */
    private enum IndexingState {
        START,
        SEARCHING_FORWARD,  // Moving forward to find the un-pressed edge of the switch
        SEARCHING_BACKWARD, // Moving backward to find the pressed edge again
        MOVING_TO_OFFSET,   // Moving to the final calculated zero position
        HOMED,              // Homing is complete and successful
        DONE                // Intermediate state before HOMED to finalize motor settings
    }

    //==================================================================================================
    //  C O N S T R U C T O R
    //==================================================================================================

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");
        indexSwitch = hardwareMap.get(TouchSensor.class, "index_switch");
        // It's good practice to set a direction. You might need to change this to FORWARD.
        spindexer.setDirection(DcMotorEx.Direction.REVERSE);
        resetEncoder(); // Reset encoder to a known state on startup
    }

    //==================================================================================================
    //  I N I T I A L I Z A T I O N   &   S T A T E   M A C H I N E
    //==================================================================================================

    /**
     * Executes the stateful homing sequence to find a precise zero position using the index switch.
     * This method must be called repeatedly in a loop (e.g., in an OpMode's init_loop).
     *
     * @return True when the entire homing process is complete; otherwise false.
     */
    public boolean doInitPosition() {
        switch (indexingState) {
            case START:
                // If not pressed, reverse until the switch is triggered.
                if (!isIndexSwitchPressed()) {
                    spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexer.setVelocity(-SPINDEXER_VELOCITY_LIMIT / 8); // Very slow reverse
                    indexingState = IndexingState.SEARCHING_BACKWARD;
                } else {
                    // If already pressed, move forward to find the edge where it releases.
                    spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT / 4); // Slow forward search
                    indexingState = IndexingState.SEARCHING_FORWARD;
                }
                return false; // Process has just begun

            case SEARCHING_FORWARD:
                // Move forward until the switch is released.
                if (!isIndexSwitchPressed()) {
                    // Switch released. Now, slowly reverse to find the precise trigger point.
                    spindexer.setVelocity(-SPINDEXER_VELOCITY_LIMIT / 8);
                    indexingState = IndexingState.SEARCHING_BACKWARD;
                }
                return false;

            case SEARCHING_BACKWARD:
                // Move backward until the switch is pressed.
                if (isIndexSwitchPressed()) {
                    // Switch has just been pressed. This is our precise trigger point.
                    int triggerPosition = spindexer.getCurrentPosition();
                    int targetPosition = triggerPosition - INDEX_OFFSET_TICKS;
                    this.lastTargetPosition = targetPosition;

                    // Command the motor to move to the final offset position.
                    spindexer.setTargetPosition(targetPosition);
                    spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT / 2);
                    indexingState = IndexingState.MOVING_TO_OFFSET;
                }
                return false;

            case MOVING_TO_OFFSET:
                // Wait until the motor reaches the final offset position.
                if (isReady()) {
                    // We've arrived. Transition to DONE to finalize the state on the next loop.
                    indexingState = IndexingState.DONE;
                }
                return false;

            case DONE:
                // Finalize the homing process.
                resetEncoder(); // The current position is now our absolute zero.
                indexingState = IndexingState.HOMED;
                return true; // Return true on this loop to signal completion.

            case HOMED:
                // The spindexer is successfully homed and holding position.
                return true;
        }
        return false; // Should not be reached
    }

    //==================================================================================================
    //  C O R E   M O V E M E N T   M E T H O D S
    //==================================================================================================

    /**
     * Searches for the next slot containing a specified artifact and rotates to it.
     * If the desired color is UNKNOWN, it finds the next non-empty slot.
     *
     * @param desiredColor The {@link SlotState} color to find (e.g., ARTIFACT_GREEN, or UNKNOWN for any artifact).
     * @return True if a matching artifact was found and rotation has begun; false if no match was found after a full search.
     */
    public boolean rotateToArtifact(SlotState desiredColor) {
        // Start searching from the slot immediately after the current one.
        int startingSlot = (currentSlot + 1) % NUMBER_OF_SLOTS;

        // Loop through all slots once to find a match.
        for (int i = 0; i < NUMBER_OF_SLOTS; i++) {
            int slotToTest = (startingSlot + i) % NUMBER_OF_SLOTS;
            SlotState slotContent = slotStates[slotToTest];

            // If the slot is not empty, check if it's what we want.
            if (slotContent != SlotState.EMPTY) {
                // A match occurs if we want any artifact (UNKNOWN) or if the color matches.
                if (desiredColor == SlotState.UNKNOWN || slotContent == desiredColor) {
                    rotateToSlot(slotToTest);
                    return true; // Match found, rotation started.
                }
            }
        }
        // If the loop completes, no matching artifact was found.
        return false;
    }

    /**
     * Rotates the spindexer to a specific slot number (0, 1, 2, etc.).
     *
     * @param slotNumber The destination slot number.
     */
    public void rotateToSlot(int slotNumber) {
        if (slotNumber < 0 || slotNumber >= NUMBER_OF_SLOTS) {
            return; // Invalid slot, do nothing.
        }

        int targetPosition = (int) Math.round(slotNumber * TICKS_PER_SLOT);
        this.lastTargetPosition = targetPosition; // Store the new target

        spindexer.setTargetPosition(targetPosition);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
        currentSlot = slotNumber;
    }

    /**
     * Advances the spindexer by one slot from its current target position.
     */
    public void advanceOneSlot() {
        int newTarget = this.lastTargetPosition - (int)Math.round(TICKS_PER_SLOT);
        this.lastTargetPosition = newTarget;

        spindexer.setTargetPosition(newTarget);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
        currentSlot = (currentSlot + 1) % NUMBER_OF_SLOTS;
    }

    /**
     * Moves the spindexer back by one slot from its current target position.
     */
    public void decreaseOneSlot() {
        int newTarget = this.lastTargetPosition + (int)Math.round(TICKS_PER_SLOT);
        this.lastTargetPosition = newTarget;

        spindexer.setTargetPosition(newTarget);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);

        // This correctly handles wrapping for negative numbers (e.g., (0 - 1 + 3) % 3 = 2).
        currentSlot = (currentSlot - 1 + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
    }

    /**
     * Adjusts the spindexer's target position by a small number of ticks for fine-tuning.
     *
     * @param ticksToNudge The number of encoder ticks to move. Positive is forward, negative is backward.
     */
    public void nudgePosition(int ticksToNudge) {
        int newTarget = this.lastTargetPosition + ticksToNudge;
        this.lastTargetPosition = newTarget;

        spindexer.setTargetPosition(newTarget);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
    }

    /**
     * Gets the raw tick count on the spindexer motor.
     *
     * @return The current position of the motor.
     */
    public int getCurrentPosition(){
        return spindexer.getCurrentPosition();
    }


    //==================================================================================================
    //  S L O T   M A N A G E M E N T
    //==================================================================================================

    /**
     * Sets the state of a specific slot.
     * @param slotNumber The slot index to modify.
     * @param state The new state for the slot.
     */
    public void setSlotState(int slotNumber, SlotState state) {
        int wrappedSlotNumber = slotNumber % NUMBER_OF_SLOTS;
        slotStates[wrappedSlotNumber] = state;
    }

    /**
     * Sets the state of the slot where an artifact would be after intaking (typically one behind current).
     * @param state The state of the newly intaked artifact.
     */
    public void setIntakeSlotState(SlotState state) {
        setSlotState(currentSlot - 1, state);
    }

    /**
     * Marks the current slot as empty, typically after shooting an artifact.
     */
    public void setCurrentSlotEmpty() {
        slotStates[currentSlot] = SlotState.EMPTY;
    }

    /**
     * Sets all slots to a predefined autonomous configuration (GPP).
     */
    public void setSlotsAuto(){
        Arrays.fill(slotStates, SlotState.ARTIFACT_PURPLE);
        slotStates[0] = SlotState.ARTIFACT_GREEN;
        currentSlot = 0; // Assume we start at slot 0
    }

    /**
     * Resets the state of all slots to EMPTY.
     */
    public void setSlotsEmpty() {
        Arrays.fill(slotStates, SlotState.EMPTY);
    }

    /**
     * Returns the state of a specific slot.
     * @param slotNumber The slot index to query.
     * @return The {@link SlotState} of the specified slot.
     */
    public SlotState getSlotState(int slotNumber) {
        int wrappedSlotNumber = slotNumber % NUMBER_OF_SLOTS;
        return slotStates[wrappedSlotNumber];
    }

    /** Returns a copy of the array representing the state of all slots. */
    public SlotState[] getSlotStates() {
        return Arrays.copyOf(slotStates, slotStates.length);
    }

    /** Returns the current slot index. */
    public int getCurrentSlot() {
        return currentSlot;
    }

    //==================================================================================================
    //  M O T O R   C O N T R O L   &   U T I L I T I E S
    //==================================================================================================

    /**
     * Checks if the motor has reached its target position within tolerance.
     *
     * @return True if the motor is at its target, false if it is still moving.
     */
    public boolean isReady() {
        int error = Math.abs(spindexer.getTargetPosition() - spindexer.getCurrentPosition());
        return error <= POSITION_TOLERANCE;
    }

    /**
     * Puts the motor into BRAKE mode, causing it to actively hold its last commanded position.
     */
    public void setBrake() {
        spindexer.setTargetPosition(this.lastTargetPosition);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
    }

    /**
     * Puts the motor into FLOAT mode, allowing it to spin freely without resistance.
     * The motor's position is still tracked.
     */
    public void setFloat() {
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(0);
    }

    /**
     * Stops all motor movement immediately by setting its power to zero.
     * This is an emergency stop and may not hold position.
     */
    public void stop() {
        spindexer.setPower(0);
    }

    /**
     * Resets the motor's encoder count to zero and sets the current position as the new zero.
     */
    private void resetEncoder() {
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
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);

        // ...// Apply the defined PIDF coefficients to the motor for RUN_TO_POSITION mode
        spindexer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SPINDEXER_PIDF);

        // Set the motor to use the RUN_TO_POSITION mode.
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    /**
     * Checks if the magnetic index switch is currently triggered.
     * The logic is inverted because the hardware sensor is normally closed.
     *
     * @return True if the switch is pressed (magnet is present).
     */
    public boolean isIndexSwitchPressed() {
        return !indexSwitch.isPressed();
    }

    /**
     * Allows for manual spinning of the spindexer with a given power.
     * @param power The power to apply, from -1.0 to 1.0.
     */
    public void spin(double power) {
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(power);
    }
}
