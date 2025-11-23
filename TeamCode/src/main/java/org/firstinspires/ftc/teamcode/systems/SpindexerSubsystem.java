package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.SlotState;
import org.firstinspires.ftc.teamcode.common.SpindexerIndex;

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
    /**
     * The number of ticks to move backward after the index switch is released to center a slot.
     */
    public static int INDEX_OFFSET_TICKS = 320;

    /**
     * This value protects the spindexer from jamming and/or crushing the world
     */
    private static final double OVERLOAD_AMPS = 7.0;

    /**
     * Ticks per revolution for the GoBilda 43 RPM motor (3895.9) geared up.
     */
    private static final double TICKS_PER_REV = 8192.0;
    /**
     * The number of slots in the spindexer.
     */
    private static final int NUMBER_OF_SLOTS = 3;
    /**
     * The number of encoder ticks needed to move one slot.
     */
    private static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    /**
     * The tolerance, in ticks, for considering the motor to have reached its target position.
     */
    private static final int POSITION_TOLERANCE = 10;

    /**
     * The maximum power limit for spindexer rotation.
     */
    public static double SPINDEXER_POWER_LIMIT = .98;
    /**
     * The maximum velocity (in ticks/sec) for spindexer rotation in RUN_TO_POSITION mode.
     */
    //public static double SPINDEXER_VELOCITY_LIMIT = 2600;

    public static double SPINDEXER_KI_TICK_RANGE = 50;

    /**
     * PIDF coefficients for position control, tunable via FTC-Dashboard.
     */
    // TODO: More tuning needed
    //(.55, 0,.0001, 10)
    // this full line works pretty good with the motor controller
    //public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(0.26, 4.26, 0, 12.6);  //10, 2,1.2, 1); 8, 4,0.2, 1
    //public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(02.6, .1, 0.1, 12.6);  //10, 2,1.2, 1); 8, 4,0.2, 1
    public static double KP = 0.001;
    public static double KI = 0.005; //.05;
    public static double KD = 0.00001;
    public static double KF = 0;
    public static double PIDF_BOUNDS = 25;

    private static final int SLOT_0_OFFSET = 0;
    private static final int SLOT_1_OFFSET = 70;
    private static final int SLOT_2_OFFSET = 50;


    // Per slot offsets
    private static final int[] SLOT_OFFSET_TICKS = {
            SLOT_0_OFFSET,    // Offset for Slot 0
            SLOT_1_OFFSET,  // Example: Slot 1 needs to be nudged back 15 ticks
            SLOT_2_OFFSET    // Example: Slot 2 needs to be nudged forward 10 ticks
    };


    public PIDFController spindexerPidf;

    private final ElapsedTime currentSpikeTimer = new ElapsedTime();

    /**
     * allows brief spikes in the current to be ignored
     */
    private static final long CURRENT_SPIKE_TIMEOUT_MS = 100;

    private static final double MIN_SORT_TIME_MS = 1000;

    private static final double MIN_ROTATE_TIME_MS = 1000;

    //==================================================================================================
    //  P R I V A T E   M E M B E R   V A R I A B L E S
    //==================================================================================================

    private final DcMotorEx spindexer;
    private final TouchSensor indexSwitch;

    /**
     * An array to hold the state of each slot (e.g., EMPTY, ARTIFACT_GREEN).
     */
    private final SlotState[] slotStates = new SlotState[NUMBER_OF_SLOTS];

    /**
     * The current slot number that is aligned with the shooting or intake mechanism.
     */
    private int curShootSlot = 0;

    private int previousSlot = -1;

    /**
     * Stores the last commanded target position, used to resume position after floating.
     */
    private int lastTargetPosition;

    /**
     * The current state of the homing/initialization process.
     */
    private HomingState homingState = HomingState.START;

    private boolean isEmergencyStop = false;

    private SortingState sortingState = SortingState.START;

    private final ElapsedTime sortingTimer = new ElapsedTime();
    private final ElapsedTime minSorterTimer = new ElapsedTime();

    private final ElapsedTime minHomeTimer = new ElapsedTime();

    private final ElapsedTime minRotateTimer = new ElapsedTime();

    private static final double MIN_HOME_TIME_MS = 2500;

    private static final double MAX_HOME_TIME_MS = 8000;

    private static final double MAX_SORT_TIME_SECOND = 20;

    private boolean isFloat = false;

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

    TelemetryManager telemetryM;
    //==================================================================================================
    //  C O N S T R U C T O R
    //==================================================================================================

    public SpindexerSubsystem(HardwareMap hardwareMap) {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");
        indexSwitch = hardwareMap.get(TouchSensor.class, "index_switch");
        spindexerPidf = new PIDFController(KP, KI, KD, KF);
        spindexerPidf.setIntegrationBounds(-PIDF_BOUNDS, PIDF_BOUNDS);
        minSorterTimer.reset();
        spindexer.setDirection(DcMotorEx.Direction.FORWARD);
        // configures the overload protection amparage
        spindexer.setCurrentAlert(OVERLOAD_AMPS, CurrentUnit.AMPS);
        spindexer.setTargetPositionTolerance(POSITION_TOLERANCE);
        //resetEncoder(); // Reset encoder to a known state on startup
    }

    public void update() {
        updatePidf();
        if (isReady() && homingState.equals(HomingState.HOMED)) {
            SpindexerIndex.setPosition(spindexer.getCurrentPosition(), curShootSlot);
        }
        else if (homingState.equals(HomingState.HOMED)) {
            SpindexerIndex.setInvalid();
        }
    }

    private void updatePidf() {
        if (isEmergencyStop || isFloat) {
            return;
        }

        // Don't do anything if the homing process is not complete.
        if (!homingState.equals(HomingState.HOMED)
                && !homingState.equals(HomingState.MOVING_TO_OFFSET)) {// Set the target for the PIDF controller
            return;
        }

        int adjustedPose = lastTargetPosition - SLOT_OFFSET_TICKS[curShootSlot];

        // Set the target for the PIDF controller
        spindexerPidf.setSetPoint(adjustedPose);

        // Only apply KI when close to setpoint
        if (Math.abs(spindexer.getCurrentPosition() - adjustedPose) > SPINDEXER_KI_TICK_RANGE) {
            spindexerPidf.clearTotalError();
        }

        // Calculate the power needed to reach the target position
        double power = -spindexerPidf.calculate(spindexer.getCurrentPosition());

        if (Math.abs(adjustedPose - spindexer.getCurrentPosition()) >= POSITION_TOLERANCE + 1) {
            // Apply the calculated power to the motor, respecting the power limit
            spindexer.setPower(Math.max(-SPINDEXER_POWER_LIMIT, Math.min(power, SPINDEXER_POWER_LIMIT)));
        }
        else {
            spindexer.setPower(0);
        }

        // You can add telemetry here to monitor PID performance
        telemetryM.addData("Spindexer Target", adjustedPose);
        telemetryM.addData("Spindexer Position", spindexer.getCurrentPosition());
        telemetryM.addData("Spindexer Power", spindexer.getPower());
        telemetryM.addData("PIDF Power", power);
        telemetryM.addData("PIDF Error", spindexerPidf.getPositionError());
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
    public boolean doInitPosition(boolean force) {
        // Define power levels for homing. Use lower power for precision.

        if (SpindexerIndex.getIsValid() && !force
            && homingState != HomingState.HOMED) {
            this.curShootSlot = SpindexerIndex.getShootSlot();
            this.lastTargetPosition = SpindexerIndex.getPosition();
            homingState = HomingState.HOMED;
            return true;
        }

        double fastSearchPower = 0.85;  // Power for moving toward the switch initially.
        double slowSearchPower = 0.095;  // Slow power for finding the precise switch edge.

        if (force && homingState.equals(HomingState.HOMED)) {
            SLOT_OFFSET_TICKS[0] = SLOT_0_OFFSET;
            SLOT_OFFSET_TICKS[1] = SLOT_1_OFFSET;
            SLOT_OFFSET_TICKS[2] = SLOT_2_OFFSET;
            homingState = HomingState.START;
        }

        switch (homingState) {
            case START:

                if (isIndexSwitchPressed()) {
                    // If starting pressed, move backwords quickly to find the release point.
                    spindexer.setPower(-fastSearchPower); // Positive power moves forward.
                    homingState = HomingState.SEARCHING_BACKWARD;
                } else {
                    // If not pressed, move forward SLOWLY to find the press point.
                    spindexer.setPower(slowSearchPower);
                    homingState = HomingState.SEARCHING_FORWARD;
                }

                return false;
            case SEARCHING_BACKWARD:
                // Continue moving backwords until the switch is released.
                if (!isIndexSwitchPressed()) {
                    // Switch released. Now, move forward slowly to find the precise trigger point.
                    spindexer.setPower(slowSearchPower);
                    homingState = HomingState.SEARCHING_FORWARD;
                }

                return false;

            case SEARCHING_FORWARD:
                // Continue moving forward until the switch is pressed.
                if (isIndexSwitchPressed()) {
                    // Switch has just been pressed. This is our precise trigger point.
                    spindexer.setPower(0); // Stop the motor immediately.

                    // The rest of your logic is already correct for the custom PIDF!
                    int triggerPosition = spindexer.getCurrentPosition();
                    this.lastTargetPosition = triggerPosition - INDEX_OFFSET_TICKS;

                    spindexerPidf.setSetPoint(this.lastTargetPosition);
                    homingState = HomingState.MOVING_TO_OFFSET;
                    minHomeTimer.reset();
                }

                return false;

            case MOVING_TO_OFFSET:
                // The updatePidf() method is now handling the movement.
                // We just need to check if it has reached the target.
                if ((isReady() && minHomeTimer.milliseconds() > MIN_HOME_TIME_MS)
                || minHomeTimer.milliseconds() > MAX_HOME_TIME_MS) {
                    homingState = HomingState.DONE;
                    spindexer.setPower(0); // Stop the motor.
                }

                return false;

            case DONE:
                // Finalize the homing process.
                resetEncoder(); // The current position is now our absolute zero.
                homingState = HomingState.HOMED;
                return true; // Signal completion.

            case HOMED:
                // The spindexer is successfully homed.
                return true;
        }

        return false; // Should not be reached
    }

    public boolean doCheckForArtifacts() {
        if (!this.isReady()) {
            return false;
        }

        // if all artifacts have a color then we can stop
        if (Arrays.stream(slotStates)
                .noneMatch(state -> state == SlotState.EMPTY || state == SlotState.UNKNOWN)) {
            return true;
        }

        // use this as a shortcut to set the other two artifacts
        if (getIntakeSlotState() == SlotState.ARTIFACT_GREEN) {
            setShootSlotState(SlotState.ARTIFACT_PURPLE);
            setStandbySlotState(SlotState.ARTIFACT_PURPLE);
            // just incase this is not set
            setIntakeSlotState(SlotState.ARTIFACT_GREEN);
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

        // Need to give a 1/2 second or so to allow the artifact to settle
        // and the color sensor to detect the artifact
        if (minSorterTimer.milliseconds() > MIN_SORT_TIME_MS) {
            minSorterTimer.reset();
            this.advanceOneSlot();
        }

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
        if (spindexer.isOverCurrent() && currentSpikeTimer.milliseconds() > CURRENT_SPIKE_TIMEOUT_MS) {
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
            return false;
        }

        // We will loop exactly NUMBER_OF_SLOTS times to guarantee a full, non-repeating search.
        // By starting our offset at 1, we ensure the first slot we check is the one *after* curShootSlot.
        for (int i = 1; i <= NUMBER_OF_SLOTS; i++) {
            // Calculate which slot to test by adding our loop offset to the current slot.
            // The modulo operator ensures we wrap around correctly after passing the last slot.
            int slotToTest = (curShootSlot + i) % NUMBER_OF_SLOTS;
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

        telemetryM.debug("No matching artifact found.");
        // If the loop completes without finding a match, it means no such artifact exists.
        return false;
    }

    /**
     * see if the Motif is available
     *
     * @return true for available otherwise false
     */
    public boolean isMotifAvailable() {
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

        // Calculate the shortest path from the current slot to the target slot.
        double delta = getDelta(slotNumber);
        // Calculate the absolute target position from the homed zero reference.

        // The new target is the last commanded position plus the shortest-path delta.
        int targetPosition = (int) Math.round(this.lastTargetPosition + delta);
        spindexerPidf.reset();
        minRotateTimer.reset();

        this.lastTargetPosition = targetPosition;

        // Updates the pointer to the new absolute slot.
        previousSlot = curShootSlot;
        curShootSlot = slotNumber;
    }

    private double getDelta(int destinationSlot) {

        // Calculate the difference in slots.
        double deltaInSlots =  curShootSlot - destinationSlot;

        // Find the shortest path in terms of slots (e.g., is it shorter to go from slot 0 to 2, or 0 to 1?)
        if (deltaInSlots > NUMBER_OF_SLOTS / 2.0) {
            deltaInSlots -= NUMBER_OF_SLOTS;
        } else if (deltaInSlots < -NUMBER_OF_SLOTS / 2.0) {
            deltaInSlots += NUMBER_OF_SLOTS;
        }

        return (deltaInSlots * TICKS_PER_SLOT);

    }

    /**
     * Advances the spindexer by one slot from its current target position
     * by updating the target slot index.
     */
    public void advanceOneSlot() {
        if (isEmergencyStop) {
            return;
        }

        // Calculate the next slot number, wrapping around if necessary.
        int nextSlot = (curShootSlot + 1) % NUMBER_OF_SLOTS;

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

        // Calculate the previous slot number, wrapping around correctly.
        int previousSlotIndex = ((curShootSlot - 1) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;

        // Command the spindexer to move to the new absolute slot.
        rotateToSlot(previousSlotIndex);
    }

    public int getCurrentOffset() {
        return INDEX_OFFSET_TICKS + SLOT_OFFSET_TICKS[curShootSlot];
    }

    /**
     * Move the spindexer to the next slot that isn't empty
     *
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

        SLOT_OFFSET_TICKS[curShootSlot] += ticksToNudge;
    }

    /**
     * Gets the raw tick count on the spindexer motor.
     *
     * @return The current position of the motor.
     */
    public int getCurrentPosition() {
        return spindexer.getCurrentPosition();
    }

//==================================================================================================
//  S L O T   M A N A G E M E N T
//==================================================================================================

/**
 * NEW MAPPING:
 * - Shoot Slot:   curShootSlot
 * - Standby Slot: (curShootSlot + 1) % NUMBER_OF_SLOTS
 * - Intake Slot:  (curShootSlot + 2) % NUMBER_OF_SLOTS
 */

    /**
     * Returns the current slot index aligned with the shooter.
     */
    public int getCurShootSlot() {
        return curShootSlot;
    }

    /**
     * Returns the state of a specific slot number.
     *
     * @param slotNumber The slot index to query.
     * @return The SlotState of the specified slot.
     */
    public SlotState getStateBySlotNum(int slotNumber) {
        // The modulo operator handles wrapping, and adding NUMBER_OF_SLOTS ensures the result is always positive.
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        return slotStates[wrappedSlotNumber];
    }

    /**
     * Sets the state of a specific slot number.
     *
     * @param slotNumber The slot index to modify.
     * @param state      The new state for the slot.
     */
    public void setSlotState(int slotNumber, SlotState state) {
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        slotStates[wrappedSlotNumber] = state;
    }


    /**
     * Sets the state of the slot currently in the shooting position.
     */
    private void setShootSlotState(SlotState state) {
        setSlotState(curShootSlot, state);
    }

    /**
     * Marks the current shoot slot as empty.
     */
    public void setShootSlotEmpty() {
        setShootSlotState(SlotState.EMPTY);
    }

    /**
     * Gets the state of the slot in the standby position (one slot ahead of shoot).
     */
    public SlotState getStandbySlotState() {
        return getStateBySlotNum(curShootSlot + 1);
    }

    /**
     * Sets the state of the slot in the standby position.
     */
    private void setStandbySlotState(SlotState state) {
        setSlotState(curShootSlot + 1, state);
    }

    /**
     * Gets the state of the slot in the intake position (two slots ahead of shoot).
     */
    public SlotState getIntakeSlotState() {
        return getStateBySlotNum(curShootSlot - 1);
    }

    /**
     * Sets the state of the slot in the intake position.
     */
    public void setIntakeSlotState(SlotState state) {
        setSlotState(curShootSlot - 1, state);
    }

    /**
     * Marks the intake slot as empty.
     */
    public void setIntakeSlotEmpty() {
        setIntakeSlotState(SlotState.EMPTY);
    }

    /**
     * Returns a copy of the array representing the state of all slots.
     */
    public SlotState[] getSlotStates() {
        return Arrays.copyOf(slotStates, slotStates.length);
    }

    /**
     * Returns the index of the slot that was previously in the shoot position.
     */
    public int getPreviousSlot() {
        return previousSlot;
    }

    /**
     * Resets the state of all slots to EMPTY.
     */
    public void setSlotsEmpty() {
        Arrays.fill(slotStates, SlotState.EMPTY);
    }

    /**
     * Sets all slots to a predefined autonomous configuration (GPP).
     */
    public void setSlotsAuto() {
        Arrays.fill(slotStates, SlotState.ARTIFACT_PURPLE);
        slotStates[2] = SlotState.ARTIFACT_GREEN;
        curShootSlot = 0; // Assume we start at slot 0
    }

    /**
     * gets the current artifact in the shooter slot
     *
     * @return the current artifact in the shooter slot
     */
    public SlotState getShootSlotState() {
        return getStateBySlotNum(curShootSlot);
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
            return true;
        }

        // A floating motor is never "ready" to hold a position.
        if (isFloat) {
            return false;
        }

        // Calculate the true final target, exactly as it's calculated in the update() method.
        // This is the position the PIDF controller is actually aiming for.
        int finalTargetPosition = lastTargetPosition - SLOT_OFFSET_TICKS[curShootSlot];

        // Calculate the error: the difference between where we want to be and where we are.
        double error = finalTargetPosition - spindexer.getCurrentPosition();

        // Check all conditions for readiness:
        //    - The position error is within the acceptable tolerance.
        //    - The minimum time for rotation has passed (to prevent premature triggers).
        return Math.abs(error) < POSITION_TOLERANCE
                && minRotateTimer.milliseconds() > MIN_ROTATE_TIME_MS;
    }

    /**
     * Puts the motor into BRAKE mode, causing it to actively hold its last commanded position.
     */
    public void setBrake() {
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        isFloat = false;
    }

    /**
     * Puts the motor into FLOAT mode, allowing it to spin freely without resistance.
     * The motor's position is still tracked.
     */
    public void setFloat() {
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        spindexer.setPower(0);
        isFloat = true;
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
        // Stop the motor and reset the encoder's internal counter to 0.
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set our internal target and the controller's target to the new zero position.
        this.lastTargetPosition = 0;
        spindexerPidf.reset();
        spindexerPidf.setSetPoint(0);

        // Set the motor for our manual control loop. The update() method will control power.
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }}
