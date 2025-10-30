package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    public static int INDEX_OFFSET_TICKS = 120;

    /** This value protects the spindexer from jamming and or crushing the world */
    private static final double OVERLOAD_AMPS = 8.0;

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

    private final ElapsedTime currentSpikeTimer = new ElapsedTime();

    /** allows brief spikes in the current to be ignored */
    private static final long CURRENT_SPIKE_TIMEOUT_MS = 100;



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
    private HomingState homingState = HomingState.START;
    private boolean isEmergencyStop = false;

    private SortingState sortingState = SortingState.START;

    private ElapsedTime sortingTimer = new ElapsedTime();

    private static final double MAX_SORT_TIME_SECOND = 20;

    /**
     * State machine enum to manage the multi-step homing process.
     */
    private enum HomingState {
        START,
        SEARCHING_FORWARD,  // Moving forward to find the un-pressed edge of the switch
        SEARCHING_BACKWARD, // Moving backward to find the pressed edge again
        MOVING_TO_OFFSET,   // Moving to the final calculated zero position
        HOMED,              // Homing is complete and successful
        DONE                // Intermediate state before HOMED to finalize motor settings
    }

    private enum SortingState {
        START,
        SEARCHING,
        DONE
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
        switch (homingState) {
            case START:
                // If not pressed, reverse until the switch is triggered.
                if (!isIndexSwitchPressed()) {
                    spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexer.setVelocity(-SPINDEXER_VELOCITY_LIMIT / 8); // Very slow reverse
                    homingState = HomingState.SEARCHING_BACKWARD;
                } else {
                    // If already pressed, move forward to find the edge where it releases.
                    spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT / 4); // Slow forward search
                    homingState = HomingState.SEARCHING_FORWARD;
                }
                return false; // Process has just begun

            case SEARCHING_FORWARD:
                // Move forward until the switch is released.
                if (!isIndexSwitchPressed()) {
                    // Switch released. Now, slowly reverse to find the precise trigger point.
                    spindexer.setVelocity(-SPINDEXER_VELOCITY_LIMIT / 8);
                    homingState = HomingState.SEARCHING_BACKWARD;
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
                    homingState = HomingState.MOVING_TO_OFFSET;
                }
                return false;

            case MOVING_TO_OFFSET:
                // Wait until the motor reaches the final offset position.
                if (isReady()) {
                    // We've arrived. Transition to DONE to finalize the state on the next loop.
                    homingState = HomingState.DONE;
                }
                return false;

            case DONE:
                // Finalize the homing process.
                resetEncoder(); // The current position is now our absolute zero.
                homingState = HomingState.HOMED;
                return true; // Return true on this loop to signal completion.

            case HOMED:
                // The spindexer is successfully homed and holding position.
                return true;
        }

        return false; // Should not be reached
    }

    public boolean doCheckForArtifacts() {
        if (spindexer.isBusy()) {
            return false;
        }

        // if all artifacts are found to be not empty nore UNKNOWN we can stop spinning.
        if(Arrays.stream(slotStates)
                .allMatch(state ->  state != SlotState.EMPTY && state != SlotState.UNKNOWN)){
            return true;
        }

        // Assume the spindexer is empty to shortcut a spin cycle
        if (getIntakeSlotState() == SlotState.EMPTY) {
            return true;
        }

        // use this as a shortcut to set the other two artifacts
        if (getIntakeSlotState() == SlotState.ARTIFACT_GREEN) {
            setShootSlotState(SlotState.ARTIFACT_PURPLE);
            setStandbySlotState(SlotState.ARTIFACT_PURPLE);
            return true;
        }

        switch (sortingState) {
            case START:
                sortingTimer.reset();
                sortingState = SortingState.SEARCHING;
                return false;
            case SEARCHING:
                if (sortingTimer.seconds() > MAX_SORT_TIME_SECOND) {
                    sortingState = SortingState.DONE;
                }

                break;
            case DONE:
                return true;
        }

        this.advanceOneSlot();
        return false;
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
        if(spindexer.isOverCurrent() && currentSpikeTimer.milliseconds() > CURRENT_SPIKE_TIMEOUT_MS){
           return true;
        }

        currentSpikeTimer.reset();
        return false;
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
     * see if the Motif is available
     * @return true for available otherwise false
     */
    public boolean isMotifAvailable()
    {
        if (!isFull()) {
            return false;
        }

        int greenCnt = 0;
        int purpleCnt = 0;

        for (SlotState state : slotStates) {
            if (state == SlotState.ARTIFACT_GREEN) {
                greenCnt++;
            } else if (state == SlotState.ARTIFACT_PURPLE) {
                purpleCnt++;
            }
        }

        return greenCnt == 1 && purpleCnt == 2;
    }

    /**
     * Rotates the spindexer to a specific slot number (0, 1, 2, etc.).
     * This is now the primary method for all spindexer rotation.
     ** @param slotNumber The destination slot number.
     */
    public void rotateToSlot(int slotNumber) {
        if (isEmergencyStop) {
            return;
        }

        if (slotNumber < 0 || slotNumber >= NUMBER_OF_SLOTS) {
            return; // Invalid slot, do nothing.
        }

        // The total number of ticks in one full revolution of the spindexer.
        final double totalTicksInCircle = NUMBER_OF_SLOTS * TICKS_PER_SLOT;
        double delta = getDelta(slotNumber, totalTicksInCircle);

        // The new target is the last commanded position plus the shortest-path delta.
        int targetPosition = (int) Math.round(this.lastTargetPosition + delta);

        // This line where the caret was is correct.
        this.lastTargetPosition = targetPosition;

        spindexer.setTargetPosition(targetPosition);
        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);

        // Updates the pointer to the new absolute slot.
        previousSlot = curShootSlot;
        curShootSlot = slotNumber;
    }

    private double getDelta(int slotNumber, double totalTicksInCircle) {
        final double halfCircle = totalTicksInCircle / 2.0;

        // Get the current motor position, normalized to a single rotation (0 to totalTicksInCircle).
        double currentPositionTicks = this.lastTargetPosition % totalTicksInCircle;
        if (currentPositionTicks < 0) {
            currentPositionTicks += totalTicksInCircle;
        }

        // Calculate the absolute destination position in ticks.
        double destinationTicks = slotNumber * TICKS_PER_SLOT;

        // Calculate the simple, direct difference between destination and current.
        double delta = destinationTicks - currentPositionTicks;

        // Correct the delta to find the shortest path by checking the distance in the opposite direction.
        // If the absolute delta is greater than half a circle, going the other way is shorter.
        if (delta > halfCircle) {
            // It's shorter to go backward (negative).
            delta -= totalTicksInCircle;
        } else if (delta < -halfCircle) {
            // It's shorter to go forward (positive).
            delta += totalTicksInCircle;
        }
        return delta;
    }

    /**
     * Advances the spindexer by one slot from its current target position
     * by updating the target slot index.
     */
    public void advanceOneSlot() {
        if (isEmergencyStop) {
            return;
        }

        // DO NOT SHIFT THE ARRAY.
        // shiftSlotStates(true); // REMOVE THIS.

        // Calculate the next slot number, wrapping around if necessary.
        int nextSlot = ((curShootSlot - 1) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;

        // Command the spindexer to move to the new absolute slot.
        // This also updates previousSlot and curShootSlot inside rotateToSlot.
        rotateToSlot(nextSlot);
    }

    /**
     * Moves the spindexer back by one slot from its current target position
     * by updating the target slot index.
     */
    public void decreaseOneSlot() {
        if (isEmergencyStop) {
            return;
        }

        // DO NOT SHIFT THE ARRAY.
        // shiftSlotStates(false); // REMOVE THIS.

        // Calculate the previous slot number, wrapping around correctly.
        int previousSlotIndex = (curShootSlot + 1) % NUMBER_OF_SLOTS;

        // Command the spindexer to move to the new absolute slot.
        rotateToSlot(previousSlotIndex);
    }

    /**
     * Move the spindexer to the next slot that isn't empty
     * @return true if slot is found otherwise false
     */
    public boolean moveToNextAvailableSlot() {
        if (isEmergencyStop) {
            return false;
        }

        for (int i = 0; i < NUMBER_OF_SLOTS; i++) {
            if (slotStates[(curShootSlot + i) % NUMBER_OF_SLOTS] != SlotState.EMPTY) {
                rotateToSlot((curShootSlot + i) % NUMBER_OF_SLOTS);
                return true;
            }
        }

        return false;
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
        return getStateBySlotNum(curShootSlot + 1);
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
        setSlotState(curShootSlot + 1, state);
    }

    private void setShootSlotState(SlotState state) {
        setSlotState(curShootSlot, state);
    }

    private void setStandbySlotState(SlotState state) {
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
    public SlotState getStateBySlotNum(int slotNumber) {
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        return slotStates[wrappedSlotNumber];
    }

    /**
     * gets the current artifact in the shooter slot
     * @return the current artifact in the shooter slot
     */
    public SlotState getShootSlotState() {
        return getStateBySlotNum(curShootSlot);
    }

    public SlotState getStandbySlotState() {
        // Corrected to point to the slot AFTER the shooter
        return getStateBySlotNum(curShootSlot - 1);
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
     * Manually shifts all slot states in the array to simulate the physical rotation
     * of the spindexer. This should be called in conjunction with any method that physically
     * indexes the spindexer.
     * @param isForward True if advancing one slot (e.g., via advanceOneSlot), false if decreasing.
     */
    private void shiftSlotStates(boolean isForward) {
        // If the array is empty or has only one slot, there's nothing to shift.
        if (slotStates.length < 2) {
            return;
        }

        if (isForward) {
            // Correct Logic for Advancing:
            // The state from the first slot moves to the last slot, simulating
            // the artifact at the intake moving towards the shooter.
            // This is a LEFT shift of the array contents.
            SlotState firstState = slotStates[0];
            System.arraycopy(slotStates, 1, slotStates, 0, NUMBER_OF_SLOTS - 1);
            slotStates[NUMBER_OF_SLOTS - 1] = firstState;
        } else {
            // Correct Logic for Decreasing:
            // The state from the last slot moves to the first slot.
            // This is a RIGHT shift of the array contents.
            SlotState lastState = slotStates[NUMBER_OF_SLOTS - 1];
            System.arraycopy(slotStates, 0, slotStates, 1, NUMBER_OF_SLOTS - 1);
            slotStates[0] = lastState;
        }
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
