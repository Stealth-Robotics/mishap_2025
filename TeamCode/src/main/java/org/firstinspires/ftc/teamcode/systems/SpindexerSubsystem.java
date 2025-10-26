package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.SlotState;

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
    /** The number of ticks to move backward after the index switch is released to center a slot. */
    public static int INDEX_OFFSET_TICKS = -250;

    /** This value protects the spindexer from jamming and or crushing the world */
    private static final double OVERLOAD_AMPS = 7.0;

    /** Ticks per revolution for the GoBilda 43 RPM motor (3895.9) geared up. */
    private static final double TICKS_PER_REV = 3895.9;
    /** The number of slots in the spindexer. */
    private static final int NUMBER_OF_SLOTS = 3;
    /** The number of encoder ticks needed to move one slot. */
    private static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    /** The tolerance, in ticks, for considering the motor to have reached its target position. */
    private static final int POSITION_TOLERANCE = 2;

    /** The maximum power limit for spindexer rotation. */
    public static double SPINDEXER_POWER_LIMIT = .5;
    /** The maximum velocity (in ticks/sec) for spindexer rotation in RUN_TO_POSITION mode. */
    public static double SPINDEXER_VELOCITY_LIMIT = 2600;

    /** PIDF coefficients for position control, tunable via FTC-Dashboard. */
    // TODO: More tuning needed
    public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(6, 4,0.2, 1);

    //==================================================================================================
    //  P R I V A T E   M E M B E R   V A R I A B L E S
    //==================================================================================================

    private final DcMotorEx spindexer;
    private final TouchSensor indexSwitch;

    /** An array to hold the state of each slot (e.g., EMPTY, ARTIFACT_GREEN). */
    private final SlotState[] slotStates = new SlotState[NUMBER_OF_SLOTS];

    /** The current slot number that is aligned with the shooting or intake mechanism. */
    private int curShootSlot = 0;

    private int previousSlot = -1;

    /** Stores the last commanded target position, used to resume position after floating. */
    private int lastTargetPosition;

    /** The current state of the homing/initialization process. */
    private IndexingState indexingState = IndexingState.START;
    private boolean isEmergencyStop = false;



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

        // The spindexer forward direction is reversed on the motor.
        spindexer.setDirection(DcMotorEx.Direction.REVERSE);
        // configures the overload protection amparage
        spindexer.setCurrentAlert(OVERLOAD_AMPS, CurrentUnit.AMPS);
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
     * Checks if the spindexer is currently over the current limit.
     * Mostlikely an artifact or hand stucking in the indexer
     *
     * @return true if something is jamming the spindexer
     */
    public boolean isJammed() {
        return spindexer.isOverCurrent();
    }

    /**
     * Searches for the next slot containing a specified artifact and rotates to it.
     * If the desired color is UNKNOWN, it finds the next non-empty slot.
     *
     * @param desiredColor The {@link SlotState} color to find (e.g., ARTIFACT_GREEN, or UNKNOWN for any artifact).
     * @return True if a matching artifact was found and rotation has begun; false if no match was found after a full search.
     */
    public boolean rotateToArtifact(SlotState desiredColor) {
        if (isEmergencyStop) {
            return true;
        }

        // Start searching from the current slot.)
        // Start searching from the slot immediately after the current one.
        int startingSlot = curShootSlot;

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
        if (isEmergencyStop) {
            return;
        }

        if (slotNumber < 0 || slotNumber >= NUMBER_OF_SLOTS) {
            return; // Invalid slot, do nothing.
        }

        int targetPosition = (int) Math.round(slotNumber * TICKS_PER_SLOT);
        this.lastTargetPosition = targetPosition; // Store the new target

        spindexer.setTargetPosition(targetPosition);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
        previousSlot = curShootSlot;
        curShootSlot = slotNumber;
    }

    /**
     * Advances the spindexer by one slot from its current target position.
     */
    public void advanceOneSlot() {
        if (isEmergencyStop) {
            return;
        }

        int newTarget = this.lastTargetPosition - (int)Math.round(TICKS_PER_SLOT);
        this.lastTargetPosition = newTarget;

        spindexer.setTargetPosition(newTarget);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
        previousSlot = curShootSlot;
        curShootSlot = (curShootSlot + 1) % NUMBER_OF_SLOTS;
    }

    /**
     * Moves the spindexer back by one slot from its current target position.
     */
    public void decreaseOneSlot() {
        if (isEmergencyStop) {
            return;
        }

        int newTarget = this.lastTargetPosition + (int)Math.round(TICKS_PER_SLOT);
        this.lastTargetPosition = newTarget;

        spindexer.setTargetPosition(newTarget);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);

        previousSlot = curShootSlot;
        // This correctly handles wrapping for negative numbers (e.g., (0 - 1 + 3) % 3 = 2).
        curShootSlot = (curShootSlot - 1 + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
    }


    /**
     * Adjusts the spindexer's target position by a small number of ticks for fine-tuning.
     *
     * @param ticksToNudge The number of encoder ticks to move. Positive is forward, negative is backward.
     */
    public void nudgePosition(int ticksToNudge) {
        if (isEmergencyStop) {
            return;
        }

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

    public int getPreviousSlot() {
        return previousSlot;
    }

    /**
     * Gets the state of the slot where an artifact would be after intaking.
     * @return The {@link SlotState} of the newly intaked artifact.
     */
    public SlotState getIntakeSlotState() {
        return getShootSlotState(curShootSlot - 1);
    }

    /**
     * Sets the state of a specific slot.
     * @param slotNumber The slot index to modify.
     * @param state The new state for the slot.
     */
    public void setSlotState(int slotNumber, SlotState state) {
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        slotStates[wrappedSlotNumber] = state;
    }

    /**
     * Sets the state of the slot where an artifact would be after intaking (typically one behind current).
     * @param state The state of the newly intaked artifact.
     */
    public void setIntakeSlotState(SlotState state) {
        setSlotState(curShootSlot - 1, state);
    }

    /**
     * Marks the intake slot as empty incase it gets ejected
     */
    public void setIntakeSlotEmpty() {
        setIntakeSlotState(SlotState.EMPTY);
    }


    /**
     * Marks the current slot as empty, typically after shooting an artifact.
     */
    public void setShootSlotEmpty() {
        slotStates[curShootSlot] = SlotState.EMPTY;
    }

    /**
     * Sets all slots to a predefined autonomous configuration (GPP).
     */
    public void setSlotsAuto(){
        Arrays.fill(slotStates, SlotState.ARTIFACT_PURPLE);
        slotStates[0] = SlotState.ARTIFACT_GREEN;
        curShootSlot = 0; // Assume we start at slot 0
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
    public SlotState getShootSlotState(int slotNumber) {
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        return slotStates[wrappedSlotNumber];
    }

    public SlotState getShootSlotState() {
        return slotStates[curShootSlot];
    }


    /** Returns a copy of the array representing the state of all slots. */
    public SlotState[] getSlotStates() {
        return Arrays.copyOf(slotStates, slotStates.length);
    }

    /** Returns the current slot index. */
    public int getCurShootSlot() {
        return curShootSlot;
    }

    //==================================================================================================
    //  M O T O R   C O N T R O L   &   U T I L I T I E S
    //==================================================================================================

    /**
     * This is a call to make when the world is on fire and
     * there is no other option to prevent bot dieing
     */
    public void setEmergencyStop() {
        this.setFloat();
        this.isEmergencyStop = true;
    }

    /**
     * Checks if the motor has reached its target position within tolerance.
     *
     * @return True if the motor is at its target, false if it is still moving.
     */
    public boolean isReady() {
        if (isEmergencyStop) {
            return false;
        }

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

    public boolean isFull() {
        return Arrays.stream(slotStates).noneMatch(state -> state == SlotState.EMPTY);
    }

    public boolean isEmpty() {
        return Arrays.stream(slotStates).allMatch(state -> state == SlotState.EMPTY);
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
        spindexer.setPower(SPINDEXER_POWER_LIMIT);

        // ...// Apply the defined PIDF coefficients to the motor for RUN_TO_POSITION mode
        spindexer.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, SPINDEXER_PIDF);

        // Set the motor to use the RUN_TO_POSITION mode.
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
