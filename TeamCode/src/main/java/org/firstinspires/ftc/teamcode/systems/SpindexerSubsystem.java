package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
     * These values adjust the offset of the slots from the 0 position of the REV through bore enc
     * the per slot values correct for hardware differences in each slot
     */

    /** Adjusts total tick starting offset all slots have this value applied to them **/
    public static int INDEX_OFFSET_TICKS = 126;
    /** slot 0 (aka 1) offset should be 0 since INDEX_OFFSET_TICKS will align this slot **/
    private static final int SLOT_0_OFFSET = 0;
    /** slot 1 (aka 2) offset seems to be a litle higher **/
    private static final int SLOT_1_OFFSET = -4;
    /** slot 2 (aka 3) offset seems to be a little lower **/
    private static final int SLOT_2_OFFSET = 8;

    /** Number of ticks to roll the spindexer forward while intaking **/
    private static final int INTAKING_OFFSET_TICKS = 50;
    /**
     * The tolerance, in ticks, for considering the motor to have reached its target position.
     */
    private static final int POSITION_TOLERANCE = 3;

    /**
     * Per Slot PID values. Slot 1 seems stickiest while slot 2 loose
     */
    public static double[] KP = {0.0092, 0.0099, 0.0077};
    public static double[] KI = {0.05,  0.055,  0.048};
    public static double[] KD = {0.0008, 0.0008, 0.0008};

    /**
     * This value protects the spindexer from jamming and/or crushing the world.
     */
    private static final double OVERLOAD_AMPS = 7.5;

    /**
     * Allows brief spikes in the current to be ignored.
     */
    private static final long CURRENT_SPIKE_TIMEOUT_MS = 400;

    /**
     * The number of absolute ticks of the REV encoder per revolution.
     */
    private static final double TICKS_PER_REV = 1024;

    /**
     * The number of slots in the spindexer.
     */
    private static final int NUMBER_OF_SLOTS = 3;

    /**
     * The number of encoder ticks needed to move one slot.
     */
    private static final double TICKS_PER_SLOT = TICKS_PER_REV / NUMBER_OF_SLOTS;

    /**
     * The maximum power limit for spindexer rotation.
     */
    public static double SPINDEXER_POWER_LIMIT = .98;

    private final PIDController spindexerPid = new PIDController(KP[0], KI[0], KD[0]);

    /** this is the maximum error build up for KI **/
    public static double PIDF_BOUNDS = .88;

    /** Maximum tick range to start applying KI **/
    public static double SPINDEXER_KI_TICK_RANGE = 15;

    /** OctoQuad PWM channel number for the REV Through Bore Encoder. */
    private static final int REV_PWM_1 = 4;

    /** REV V1 Through Bore Encoder settings **/
    private static final int REV_PWM_LOW = 1;
    private static final int REV_PWM_HIGH = 1024;
    private static final double DEGREES_PER_US = (360.0 / 1024.0);
    private static final int VELOCITY_SAMPLE_INTERVAL_MS = 25;   // To provide 40 updates/Sec.

    /** Per slot offsets in ticks positive value moves slot up, negative value moves slot down **/
    private static final int[] SLOT_OFFSET_TICKS = {
            SLOT_0_OFFSET,
            SLOT_1_OFFSET,
            SLOT_2_OFFSET
    };

    private static final double MIN_SORT_TIME_MS = 1500;
    private static final double MIN_ROTATE_TIME_MS = 1000;
    private static final double MIN_HOME_TIME_MS = 1500;
    private static final double MAX_HOME_TIME_MS = 8000;
    private static final double MAX_SORT_TIME_SECOND = 10;

    //==================================================================================================
    //  D E P E N D E N C I E S
    //==================================================================================================

    private final DcMotorEx spindexer;
    private final OctoQuad octoquad;
    private final TelemetryManager telemetryM;

    //==================================================================================================
    //  S T A T E   V A R I A B L E S
    //==================================================================================================

    /** Is the robot intaking **/
    private boolean isIntaking = false;

    /**
     * An array to hold the state of each slot (e.g., EMPTY, ARTIFACT_GREEN).
     */
    private final SlotState[] slotStates = new SlotState[NUMBER_OF_SLOTS];

    /**
     * The current slot number that is aligned with the shooting or intake mechanism.
     */
    private int curShootSlot = 0;

    /**
     * Stores the last commanded target position, used to resume position after floating.
     */
    private int lastTargetPosition;

    /**
     * The current state of the homing/initialization process.
     */
    private HomingState homingState = HomingState.START;

    /**
     * The current state of the artifact sorting process.
     */
    private SortingState sortingState = SortingState.START;

    private boolean isEmergencyStop = false;
    private boolean isFloat = false;

    private final ElapsedTime currentSpikeTimer = new ElapsedTime();
    private final ElapsedTime sortingTimer = new ElapsedTime();
    private final ElapsedTime minSorterTimer = new ElapsedTime();
    private final ElapsedTime minHomeTimer = new ElapsedTime();
    private final ElapsedTime minRotateTimer = new ElapsedTime();

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

    /**
     * State machine enum to manage the artifact sorting process.
     */
    private enum SortingState {
        START,
        SEARCHING,
        DONE
    }

    //==================================================================================================
    //  C O N S T R U C T O R
    //==================================================================================================

    /**
     * Constructs a new SpindexerSubsystem.
     *
     * @param hardwareMap The HardwareMap object from the OpMode, used to get device instances.
     */
    public SpindexerSubsystem(HardwareMap hardwareMap) {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        spindexer = hardwareMap.get(DcMotorEx.class, "spindexer_motor");
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        initOctoQuad();
        spindexerPid.setIntegrationBounds(-PIDF_BOUNDS, PIDF_BOUNDS);
        minSorterTimer.reset();
        spindexer.setDirection(DcMotorEx.Direction.FORWARD);
        // configures the overload protection amperage
        spindexer.setCurrentAlert(OVERLOAD_AMPS, CurrentUnit.AMPS);
        spindexer.setTargetPositionTolerance(POSITION_TOLERANCE);
    }

    //==================================================================================================
    //  I N I T I A L I Z A T I O N
    //==================================================================================================

    /**
     * Configures the OctoQuad for quadrature encoders 0-3 and PWM (absolute encoder) for 4-7.
     */
    private void initOctoQuad() {
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);
        octoquad.setAllVelocitySampleIntervals(VELOCITY_SAMPLE_INTERVAL_MS);

        // set channels 4 to 7 for REV through bore PWM (absolute) encoders
        for (int i = 4; i < 8; i++) {
            // TODO: decide if we want to wrap or not
            octoquad.setSingleChannelPulseWidthTracksWrap(i, true);
            octoquad.setSingleChannelPulseWidthParams(i, REV_PWM_LOW, REV_PWM_HIGH);
        }

        // set the spindexer REV encoder to reverse
        octoquad.setSingleEncoderDirection(REV_PWM_1, OctoQuad.EncoderDirection.REVERSE);

        // if the octoquad has a brown out this will make sure it picks the setttings back up.
        octoquad.saveParametersToFlash();
    }

    //==================================================================================================
    //  P E R I O D I C   U P D A T E S
    //==================================================================================================

    /**
     * Main update loop method for the subsystem. Should be called repeatedly.
     * Updates the PIDF controller and caches sensor data.
     */
    public void update() {
        octoquad.refreshCache();
        resetSpindexerOffsetsFast();
        updatePidf();

        if (isReady() && homingState.equals(HomingState.HOMED)) {
            SpindexerIndex.setPosition(this.getCurrentPosition(), curShootSlot);
        } else if (homingState.equals(HomingState.HOMED)) {
            SpindexerIndex.setInvalid();
        }
    }

    /**
     * Designed to be called every loop to reset the spindexer
     * slot 0 position after x number of complete rotations to
     * help prevent rounding drift.
     */
    private void resetSpindexerOffsetsFast() {
        if (curShootSlot != 0
                || !isReady()
                || Math.abs(getCurrentPosition()) < (2 * TICKS_PER_REV)) {
            return;
        }

        this.resetOctoQuad();
        this.lastTargetPosition = INDEX_OFFSET_TICKS;
        spindexerPid.reset();
    }

    private void resetOctoQuad() {

        octoquad.resetSinglePosition(REV_PWM_1);
        // force the octoquad to reset the wrap value
        octoquad.setSingleChannelPulseWidthTracksWrap(REV_PWM_1, false);
        octoquad.setSingleChannelPulseWidthTracksWrap(REV_PWM_1, true);
        octoquad.refreshCache();
    }
    /**
     * Updates the PIDF controller calculations and applies power to the motor.
     * This method runs continuously to maintain the target position.
     */
    private void updatePidf() {
        if (isEmergencyStop || isFloat) {
            return;
        }

        // Don't do anything if the homing process is not complete.
        if (!homingState.equals(HomingState.HOMED)
                && !homingState.equals(HomingState.MOVING_TO_OFFSET)) {// Set the target for the PIDF controller
            return;
        }

        // UHG I created per slot PID because friction is very different for slot 2
        spindexerPid.setPID(KP[curShootSlot], KI[curShootSlot], KD[curShootSlot]);

        int adjustedPose = lastTargetPosition + SLOT_OFFSET_TICKS[curShootSlot];
        int currentPosition = this.getCurrentPosition();
        if (this.isIntaking) {
            adjustedPose += INTAKING_OFFSET_TICKS;
        }


        // Set the target for the PIDF controller
        spindexerPid.setSetPoint(adjustedPose);

        // Only apply KI when close to setpoint
        if (Math.abs(currentPosition - adjustedPose) > SPINDEXER_KI_TICK_RANGE) {
            spindexerPid.clearTotalError();
        }

        double power = 0;
        if (Math.abs(adjustedPose - currentPosition) + 1 >= POSITION_TOLERANCE) {
            // Apply the calculated power to the motor, respecting the power limit
            // Calculate the power needed to reach the target position
            power = spindexerPid.calculate(currentPosition);
        } else {
            spindexerPid.reset();
        }

        spindexer.setPower(Math.max(-SPINDEXER_POWER_LIMIT, Math.min(power, SPINDEXER_POWER_LIMIT)));

        // Telemetry to monitor PID performance
//        telemetryM.addData("ShootSlot", curShootSlot);
//        telemetryM.addLine(String.format("Position: %d target: %d delta %d",
//                currentPosition, adjustedPose, Math.abs(adjustedPose - currentPosition)));
//        telemetryM.addData("Spindexer Power", spindexer.getPower());
//        telemetryM.addData("PIDF Power", power);
//        telemetryM.addData("PIDF Error", spindexerPid.getPositionError());
//        telemetryM.addData("Absolute Position", this.getCurrentAbsolutePosition());
    }

    //==================================================================================================
    //  S T A T E   M A C H I N E S
    //==================================================================================================

    /**
     * Executes the stateful homing sequence to find the zero position + offset.
     * This method must be called repeatedly in a loop (e.g., in an OpMode's init_loop).
     *
     * @param force If true, forces the homing sequence to re-run even if a valid position was carried over.
     * @return True when the entire homing process is complete; otherwise false.
     */
    public boolean doInitPosition(boolean force) {

        // If a state has been carried over use that
        if (SpindexerIndex.getIsValid() && !force
                && homingState != HomingState.HOMED) {
            this.curShootSlot = SpindexerIndex.getShootSlot();
            this.lastTargetPosition = SpindexerIndex.getPosition();
            homingState = HomingState.HOMED;
            return true;
        }

        // reset any offsets added from the controller
        if (force && homingState.equals(HomingState.HOMED)) {
            SLOT_OFFSET_TICKS[0] = SLOT_0_OFFSET;
            SLOT_OFFSET_TICKS[1] = SLOT_1_OFFSET;
            SLOT_OFFSET_TICKS[2] = SLOT_2_OFFSET;
            homingState = HomingState.START;
        }

        // start a rotate to nearest slot position
        if (homingState.equals(HomingState.START)) {
            // force the octoquad to reset the wrap value
            this.resetOctoQuad();
            // Option 1
            this.curShootSlot = this.getClosestSlotNumber();
            int currentPosition = getCurrentPosition();
            double shortestPathDelta = calculateShortestPathDelta(currentPosition, getAbsoluteSloteTicks(curShootSlot), TICKS_PER_REV);

            // If you want to rotate to slot 0 for init swap this for option 1
//            this.curShootSlot = 0;
//            int currentPosition = getCurrentPosition();
//            double shortestPathDelta = calculateShortestPathDelta(currentPosition, INDEX_OFFSET_TICKS, TICKS_PER_REV);
            // Important that selected slot ticks are removed as they are added back in durring the PIDF calc
            this.lastTargetPosition = (int) Math.round(currentPosition + shortestPathDelta - SLOT_OFFSET_TICKS[curShootSlot]);
            minHomeTimer.reset();
            homingState = HomingState.MOVING_TO_OFFSET;
            return false;
        }

        if (isReady() && minHomeTimer.milliseconds() > MIN_HOME_TIME_MS
                || minHomeTimer.milliseconds() > MAX_HOME_TIME_MS) {
            homingState = HomingState.HOMED;
            return true;
        }

        return false;
    }

    /**
     * A state machine to check for artifacts.
     * Currently contains logic to auto-fill slots if a green artifact is detected.
     *
     * @return True if artifact checking is complete or not needed; false if in progress.
     */
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

        // Need to give a little time to allow the artifact to settle
        // and the color sensor to detect the artifact
        if (minSorterTimer.milliseconds() > MIN_SORT_TIME_MS) {
            minSorterTimer.reset();
            this.advanceOneSlot();
        }

        return false;
    }
    //==================================================================================================
    //  S E N S O R   &   S T A T E   M E T H O D S
    //==================================================================================================

    public void setIntaking(boolean isIntakeRunning) {
        this.isIntaking = isIntakeRunning;
    }

    /**
     * Gets the current per-slot offset from the 0 position
     * @return The current offset in ticks
     */
    public int getCurrentOffset() {
        return INDEX_OFFSET_TICKS + SLOT_OFFSET_TICKS[curShootSlot];
    }

    /**
     * Gets the raw, continuous encoder position from the motor.
     *
     * @return The current raw encoder position.
     */
    public int getCurrentPosition() {
        return octoquad.readSinglePosition_Caching(REV_PWM_1);
    }

    /**
     * Gets the "absolute" position of the encoder, wrapped within a single revolution.
     * The result is always positive.
     *
     * @return The current position within a single revolution (e.g., 1 to 1024).
     */
    public int getCurrentAbsolutePosition() {
        return Math.floorMod(getCurrentPosition(), REV_PWM_HIGH) + 1;
    }

    /**
     * Gets the current absolute angle of the spindexer in degrees.
     * @return The current absolute angle of the spindexer in degrees.
     */
    public double getCurrentAbsoluteAngle() {
        return (getCurrentAbsolutePosition() * DEGREES_PER_US);
    }

    /**
     * Checks if the motor has reached its target position within tolerance.
     *
     * @return True if the motor is at its target, false if it is still moving.
     */
    public boolean isReady() {
        if (isEmergencyStop || !homingState.equals(HomingState.HOMED)) {
            return true;
        }

        if (isFloat) {
            return false;
        }

        int finalTarget = lastTargetPosition + SLOT_OFFSET_TICKS[curShootSlot];
        if (this.isIntaking) {
            finalTarget += INTAKING_OFFSET_TICKS;
        }

        double error = finalTarget - this.getCurrentPosition();

        return Math.abs(error) <= POSITION_TOLERANCE && minRotateTimer.milliseconds() > MIN_ROTATE_TIME_MS;
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
     * Checks if the Motif is available
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


    //==================================================================================================
    //  S L O T   M A N A G E M E N T
    //==================================================================================================
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
        spindexerPid.reset();
        minRotateTimer.reset();

        this.lastTargetPosition = targetPosition;

        // Updates the pointer to the new absolute slot.
        curShootSlot = slotNumber;
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
     * Sets the state of the slot currently in the intake position.
     *
     * @param state The new {@link SlotState} for the intake slot.
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
     * Resets the state of all slots to EMPTY.
     */
    public void setSlotsEmpty() {
        Arrays.fill(slotStates, SlotState.EMPTY);
    }

    /**
     * Sets all slots to a predefined autonomous configuration (GPP).
     */
    public void setSlotsAuto() {
        slotStates[0] = SlotState.ARTIFACT_PURPLE;
        slotStates[1] = SlotState.ARTIFACT_PURPLE;
        slotStates[2] = SlotState.ARTIFACT_GREEN;
    }

    /**
     * Sets the state of a specific slot number, handling wrapping for invalid inputs.
     *
     * @param slotNumber The slot index to modify.
     * @param state      The new {@link SlotState} for the slot.
     */
    public void setSlotState(int slotNumber, SlotState state) {
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        slotStates[wrappedSlotNumber] = state;
    }

    /**
     * Sets the state of the slot currently in the shooting position.
     *
     * @param state The new {@link SlotState} for the shoot slot.
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
     * Sets the state of the slot in the standby position (one ahead of shoot).
     *
     * @param state The new {@link SlotState} for the standby slot.
     */
    private void setStandbySlotState(SlotState state) {
        setSlotState(curShootSlot + 1, state);
    }

    /**
     * Returns the current slot index aligned with the shooter.
     */
    public int getCurShootSlot() {
        return curShootSlot;
    }

    /**
     * Gets the state of the slot currently in the shooting position.
     *
     * @return The {@link SlotState} of the current shoot slot.
     */
    public SlotState getShootSlotState() {
        return getStateBySlotNum(curShootSlot);
    }

    /**
     * Gets the state of the slot in the intake position (one slot before shoot).
     *
     * @return The {@link SlotState} of the intake slot.
     */
    public SlotState getIntakeSlotState() {
        return getStateBySlotNum(curShootSlot - 1);
    }

    /**
     * Gets the state of the slot in the standby position (one slot ahead of shoot).
     *
     * @return The {@link SlotState} of the standby slot.
     */
    public SlotState getStandbySlotState() {
        return getStateBySlotNum(curShootSlot + 1);
    }

    /**
     * Returns the state of a specific slot number, handling wrapping for invalid inputs.
     *
     * @param slotNumber The slot index to query.
     * @return The {@link SlotState} of the specified slot.
     */
    public SlotState getStateBySlotNum(int slotNumber) {
        // The modulo operator handles wrapping, and adding NUMBER_OF_SLOTS ensures the result is always positive.
        int wrappedSlotNumber = ((slotNumber % NUMBER_OF_SLOTS) + NUMBER_OF_SLOTS) % NUMBER_OF_SLOTS;
        return slotStates[wrappedSlotNumber];
    }


    //==================================================================================================
    //  P R I V A T E   H E L P E R   M E T H O D S
    //==================================================================================================

    /**
     * Calculates the shortest travel distance between two slots.
     *
     * @param destinationSlot The target slot number.
     * @return The shortest delta in ticks (positive or negative) to reach the target.
     */
    private double getDelta(int destinationSlot) {

        // Calculate the difference in slots.
        double deltaInSlots = destinationSlot - curShootSlot;

        // Find the shortest path in terms of slots (e.g., is it shorter to go from slot 0 to 2, or 0 to 1?)
        if (deltaInSlots > NUMBER_OF_SLOTS / 2.0) {
            deltaInSlots -= NUMBER_OF_SLOTS;
        } else if (deltaInSlots < -NUMBER_OF_SLOTS / 2.0) {
            deltaInSlots += NUMBER_OF_SLOTS;
        }

        return (deltaInSlots * TICKS_PER_SLOT);

    }

    /**
     * Calculates the shortest travel distance between two continuous encoder positions,
     * accounting for wraparound within a single revolution.
     *
     * @param currentPosition    The starting encoder position.
     * @param targetPosition     The target encoder position.
     * @param ticksPerRevolution The total number of ticks in one full revolution.
     * @return The shortest delta in ticks (positive or negative) to reach the target.
     */
    private double calculateShortestPathDelta(double currentPosition, double targetPosition, double ticksPerRevolution) {
        // Calculate the direct path
        double directDelta = targetPosition - currentPosition;

        // Calculate the path if we wrap around (go the other way)
        double wrappedDelta;
        if (directDelta > 0) {
            // If moving forward, the wrapped path is to go backward by the remaining distance
            wrappedDelta = directDelta - ticksPerRevolution;
        } else {
            // If moving backward, the wrapped path is to go forward by the remaining distance
            wrappedDelta = directDelta + ticksPerRevolution;
        }

        // Return the delta with the smaller absolute magnitude
        return Math.abs(directDelta) <= Math.abs(wrappedDelta) ? directDelta : wrappedDelta;
    }

    /**
     * Finds the slot number that is physically closest to the spindexer's current absolute position.
     *
     * @return The index of the closest slot.
     */
    private int getClosestSlotNumber() {
        int closestSlot = 0;
        int currentPosition = this.getCurrentAbsolutePosition();
        for (int i = 0; i < NUMBER_OF_SLOTS; i++) {
            int slotTicks = getAbsoluteSloteTicks(i);
            if (Math.abs(slotTicks - currentPosition) <= TICKS_PER_SLOT / 2) {
                closestSlot = i;
                break;
            }
        }

        return closestSlot;
    }

    /**
     * Calculates the absolute encoder ticks for a given slot number's target position.
     *
     * @param slotNumber The slot number for which to calculate the ticks.
     * @return The absolute encoder ticks for the specified slot.
     */
    private int getAbsoluteSloteTicks(int slotNumber){
        return (int) Math.round(INDEX_OFFSET_TICKS + SLOT_OFFSET_TICKS[slotNumber] + TICKS_PER_SLOT * slotNumber);
    }

    //==================================================================================================
    //  M O T O R   C O N T R O L   &   U T I L I T I E S
    //==================================================================================================

    /**
     * Checks if the spindexer is currently over the current limit, indicating a jam.
     *
     * @return True if the motor current is over the limit for a specified duration.
     */
    public boolean isJammed() {
        if (spindexer.isOverCurrent()) {
            return currentSpikeTimer.milliseconds() > CURRENT_SPIKE_TIMEOUT_MS;
        }

        currentSpikeTimer.reset();
        return false;
    }

    /**
     * This is a call to make when the world is on fire and
     * there is no other option to prevent bot dieing
     */
    public void setEmergencyStop() {
        this.setFloat();
        this.isEmergencyStop = true;
    }

    public void resetEmergencyStop() {
        this.isEmergencyStop = false;
        setBrake();
    }

    /**
     * Puts the motor into BRAKE mode, causing it to actively hold its last commanded position.
     */
    public void setBrake() {
        spindexer.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        isFloat = false;
    }

    /**
     * Puts the motor into FLOAT mode, allowing it to spin freely without resistance.
     * The motor's position is still tracked.
     */
    public void setFloat() {
        spindexer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        spindexer.setPower(0);
        isFloat = true;
    }

    /**
     * Checks if all slots are full.
     *
     * @return true for full otherwise false
     */
    public boolean isFull() {
        return Arrays.stream(slotStates).noneMatch(state -> state == org.firstinspires.ftc.teamcode.common.SlotState.EMPTY);
    }


    /**
     * Checks if all slots are empty.
     *
     * @return true for empty otherwise false
     */
    public boolean isEmpty() {
        return Arrays.stream(slotStates).allMatch(state -> state == org.firstinspires.ftc.teamcode.common.SlotState.EMPTY);
    }
}
