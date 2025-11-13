package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.common.SlotState;
import org.firstinspires.ftc.teamcode.common.ZoneDistance;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

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
    public static int INDEX_OFFSET_TICKS = 330;
    public static int NEAR_ZONE_OFFSET = 0;
    public static int MID_ZONE_OFFSET = 0;
    public static int FAR_ZONE_OFFSET = 0;

    /** This value protects the spindexer from jamming and/or crushing the world */
    private static final double OVERLOAD_AMPS = 8.0;

    /** Ticks per revolution for the GoBilda 43 RPM motor (3895.9) geared up. */
    private static final double TICKS_PER_REV = 8192;
    /** The number of slots in the spindexer. */
    private static final int NUMBER_OF_SLOTS = 3;
    /** The number of encoder ticks needed to move one slot. */
    private static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    /** The tolerance, in ticks, for considering the motor to have reached its target position. */
    private static final int POSITION_TOLERANCE = 70;

    /** The maximum power limit for spindexer rotation. */
    public static double SPINDEXER_POWER_LIMIT = .9;
    /** The maximum velocity (in ticks/sec) for spindexer rotation in RUN_TO_POSITION mode. */
    public static double SPINDEXER_VELOCITY_LIMIT = 2600;

    /** PIDF coefficients for position control, tunable via FTC-Dashboard. */
    // TODO: More tuning needed
           //(.55, 0,.0001, 10)
            // this full line works pretty good with the motor controller
    //public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(0.26, 4.26, 0, 12.6);  //10, 2,1.2, 1); 8, 4,0.2, 1
    public static PIDFCoefficients SPINDEXER_PIDF = new PIDFCoefficients(2.5, 0.01,0, 10.0);  //10, 2,1.2, 1); 8, 4,0.2, 1
//    public static PIDFController SPINDEXER_PIDF_CONTROLLER = new PIDFController(
//            .026
//            , 3.5
//            , .0001
//            , SPINDEXER_PIDF.f
//    );

    private final ElapsedTime currentSpikeTimer = new ElapsedTime();

    /** allows brief spikes in the current to be ignored */
    private static final long CURRENT_SPIKE_TIMEOUT_MS = 200;

    private static final double MIN_SORT_TIME_MS = 500;

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

    private final ElapsedTime sortingTimer = new ElapsedTime();
    private final ElapsedTime minSorterTimer = new ElapsedTime();

    private final ElapsedTime minHomeTimer = new ElapsedTime();

    private static final double MIN_HOME_TIME_MS = 3500;

    private static final double MAX_SORT_TIME_SECOND = 20;

    private ZoneDistance currentZone = ZoneDistance.FAR;
    private final Map<ZoneDistance, Integer> zoneMap;

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
        zoneMap = new HashMap<>();
        zoneMap.put(ZoneDistance.NEAR, -NEAR_ZONE_OFFSET);
        zoneMap.put(ZoneDistance.MID, -MID_ZONE_OFFSET);
        zoneMap.put(ZoneDistance.FAR, -FAR_ZONE_OFFSET);
        minSorterTimer.reset();
        // The spindexer forward direction is reversed on the motor.
        spindexer.setDirection(DcMotorEx.Direction.REVERSE);
        // configures the overload protection amparage
        spindexer.setCurrentAlert(OVERLOAD_AMPS, CurrentUnit.AMPS);
        resetEncoder(); // Reset encoder to a known state on startup
    }

    public void update() {

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

        double velocity = 900;
        switch (homingState) {
            case START:
                // If not pressed, reverse until the switch is triggered.
                spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                if (isIndexSwitchPressed()) {
                    // If already pressed, move forward to find the edge where it releases.
                    spindexer.setVelocity(velocity); // Slow forward search
                    homingState = HomingState.SEARCHING_FORWARD;
                } else {
                    spindexer.setVelocity(-velocity / 4); //  slowly move forward until pressed
                    homingState = HomingState.SEARCHING_BACKWARD;
                }
                return false; // Process has just begun

            case SEARCHING_FORWARD:
                // Move forward until the switch is released.
                if (!isIndexSwitchPressed()) {
                    // Switch released. Now, slowly reverse to find the precise trigger point.
                    spindexer.setVelocity(-velocity / 4);
                    spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    homingState = HomingState.SEARCHING_BACKWARD;
                }

                return false;

            case SEARCHING_BACKWARD:
                // Move backward until the switch is pressed.
                if (isIndexSwitchPressed()) {
                    // Switch has just been pressed. This is our precise trigger point.
                    spindexer.setVelocity(0);

                    int triggerPosition = spindexer.getCurrentPosition();
                    int targetPosition = triggerPosition - INDEX_OFFSET_TICKS;
                    this.lastTargetPosition = targetPosition;

                    // Command the motor to move to the final offset position.
                    spindexer.setTargetPosition(targetPosition);
                    spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spindexer.setPower(SPINDEXER_POWER_LIMIT);
                    spindexer.setVelocity(velocity);
                    homingState = HomingState.MOVING_TO_OFFSET;
                    minHomeTimer.reset();
                }
                return false;

            case MOVING_TO_OFFSET:
                // Wait until the motor reaches the final offset position.
                if (isReady() && minHomeTimer.milliseconds() > MIN_HOME_TIME_MS) {
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
        if (!this.isReady()) {
            return false;
        }

        // if all artifacts have a color then we can stop
        if(Arrays.stream(slotStates)
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

        // The new target is the last commanded position plus the shortest-path delta.
        // Since lastTargetPosition already includes nudges, this correctly calculates
        // the next absolute motor position.
        int targetPosition = (int) Math.round(this.lastTargetPosition + delta);

        // Update the last commanded position.
        this.lastTargetPosition = targetPosition;

        // Set motor pose based on current zone offset
        //noinspection DataFlowIssue
        spindexer.setTargetPosition(targetPosition + zoneMap.get(currentZone));
//        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);

        // Updates the pointer to the new absolute slot.
        previousSlot = curShootSlot;
        curShootSlot = slotNumber;
    }

    private double getDelta(int destinationSlot) {

        // Calculate the difference in slots.
        double deltaInSlots = destinationSlot - curShootSlot;

        // Find the shortest path in terms of slots (e.g., is it shorter to go from slot 0 to 4, or 0 to -1?)
        if (deltaInSlots > NUMBER_OF_SLOTS / 2.0) {
            deltaInSlots -= NUMBER_OF_SLOTS;
        } else if (deltaInSlots < -NUMBER_OF_SLOTS / 2.0) {
            deltaInSlots += NUMBER_OF_SLOTS;
        }

        // Convert the shortest path in slots to a delta in ticks.
        return deltaInSlots * TICKS_PER_SLOT;
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

        // Calculate the previous slot number, wrapping around correctly.
        int previousSlotIndex = (curShootSlot + 1) % NUMBER_OF_SLOTS;

        // Command the spindexer to move to the new absolute slot.
        rotateToSlot(previousSlotIndex);
    }

    public void setOffsetByDistance(double distance) {
        if (isEmergencyStop) {
            return;
        }

        ZoneDistance lastZone = currentZone;
        if (distance < ZoneDistance.MID.id){
            currentZone = ZoneDistance.NEAR;
        }else if (distance < ZoneDistance.FAR.id){
            currentZone = ZoneDistance.MID;
        }else{
            currentZone = ZoneDistance.FAR;
        }

        // Move the spindexer to the correct position if a change has happend
        if (lastZone != currentZone) {
            nudgePosition(zoneMap.get(currentZone));
        }
    }

    public int getCurrentOffset() {
        //noinspection DataFlowIssue
        return INDEX_OFFSET_TICKS - zoneMap.get(currentZone);
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

        // noinspection DataFlowIssue
        int curZoneOffset = zoneMap.get(currentZone);
        curZoneOffset += ticksToNudge;
        zoneMap.put(currentZone, curZoneOffset);

        spindexer.setTargetPosition(this.lastTargetPosition + curZoneOffset);
//        spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindexer.setVelocity(SPINDEXER_VELOCITY_LIMIT);
    }

    public ZoneDistance getCurrentZone() {
        return currentZone;
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
     * Resets the motor's encoder count to zero and sets the current position as the new zero.
     */
    private void resetEncoder() {
        // Stop and reset the encoder to a known state on initialization.
        // This makes the current position '0' when the robot starts.
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set a tolerance for how close to the target is "close enough" (in encoder ticks)
        // This can help prevent oscillations around the target.
        spindexer.setTargetPositionTolerance(POSITION_TOLERANCE);

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
