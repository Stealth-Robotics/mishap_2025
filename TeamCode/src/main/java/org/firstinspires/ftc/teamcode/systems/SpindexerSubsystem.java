package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static int INDEX_OFFSET_TICKS = 128;


    private static final int SLOT_0_OFFSET = 0;
    private static final int SLOT_1_OFFSET = -2;
    private static final int SLOT_2_OFFSET = 6;

    /**
     * This value protects the spindexer from jamming and/or crushing the world
     */
    private static final double OVERLOAD_AMPS = 7.0;


    /**
     * the nuber of absolute ticks of the REV encoder per revolution
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
     * The tolerance, in ticks, for considering the motor to have reached its target position.
     */
    private static final int POSITION_TOLERANCE = 4;

    /**
     * The maximum power limit for spindexer rotation.
     */
    public static double SPINDEXER_POWER_LIMIT = .98;

    /**
     * PIDF coefficients for position control, tunable via FTC-Dashboard.
     */
    public static double KP = 0.0075;
    public static double KI = 0.09;
    public static double KD = 0.0005;
    public static double KF = 0;

    public static  PIDFController spindexerPidf =
            new PIDFController(KP, KI, KD, KF);

    public static double PIDF_BOUNDS = .85;
    public static double SPINDEXER_KI_TICK_RANGE = 15;

    private static final int REV_PWM_1 = 4;

    // REV V1 Through Bore Encoder PMW range
    private static final int REV_PWM_LOW = 1;
    private static final int REV_PWM_HIGH = 1024;
    private static final double DEGREES_PER_US = (360.0 / 1024.0);  // REV Through Bore Encoder
    private static final int VELOCITY_SAMPLE_INTERVAL_MS = 25;   // To provide 40 updates/Sec.
    private final OctoQuad octoquad;
    // Per slot offsets
    private static final int[] SLOT_OFFSET_TICKS = {
            SLOT_0_OFFSET,    // Offset for Slot 0
            SLOT_1_OFFSET,  // Example: Slot 1 needs to be nudged back 15 ticks
            SLOT_2_OFFSET    // Example: Slot 2 needs to be nudged forward 10 ticks
    };


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

    private boolean isEmergencyStop = false;

    private SortingState sortingState = SortingState.START;

    private final ElapsedTime sortingTimer = new ElapsedTime();
    private final ElapsedTime minSorterTimer = new ElapsedTime();

    private final ElapsedTime minHomeTimer = new ElapsedTime();

    private final ElapsedTime minRotateTimer = new ElapsedTime();

    private static final double MIN_HOME_TIME_MS = 1500;

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
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        initOctoQuad();
        spindexerPidf.setIntegrationBounds(-PIDF_BOUNDS, PIDF_BOUNDS);
        minSorterTimer.reset();
        spindexer.setDirection(DcMotorEx.Direction.FORWARD);
        // configures the overload protection amparage
        spindexer.setCurrentAlert(OVERLOAD_AMPS, CurrentUnit.AMPS);
        spindexer.setTargetPositionTolerance(POSITION_TOLERANCE);
        //resetEncoder(); // Reset encoder to a known state on startup
    }

    /**
     * Configures the octoquad for quadrature encoders 0-3 and PWM (absolute encoder) for 4-7.
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

    public void update() {
        updatePidf();
        if (isReady() && homingState.equals(HomingState.HOMED)) {
            octoquad.refreshCache();
            SpindexerIndex.setPosition(this.getCurrentPosition(), curShootSlot);
        } else if (homingState.equals(HomingState.HOMED)) {
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

        int adjustedPose = lastTargetPosition + SLOT_OFFSET_TICKS[curShootSlot];
        int currentPosition = this.getCurrentPosition();

        // Set the target for the PIDF controller
        spindexerPidf.setSetPoint(adjustedPose);

        // Only apply KI when close to setpoint
        if (Math.abs(currentPosition - adjustedPose) > SPINDEXER_KI_TICK_RANGE) {
            spindexerPidf.clearTotalError();
        }

        // Calculate the power needed to reach the target position
        double power = spindexerPidf.calculate(currentPosition);

        if (Math.abs(adjustedPose - currentPosition) + 1 >= POSITION_TOLERANCE) {
            // Apply the calculated power to the motor, respecting the power limit
            spindexer.setPower(Math.max(-SPINDEXER_POWER_LIMIT, Math.min(power, SPINDEXER_POWER_LIMIT)));
        } else {
            spindexer.setPower(0);
            spindexerPidf.clearTotalError();
        }

        // You can add telemetry here to monitor PID performance
        telemetryM.addData("Spindexer Target", adjustedPose);
        telemetryM.addData("Spindexer Position", currentPosition);
        telemetryM.addData("Spindexer Power", spindexer.getPower());
        telemetryM.addData("PIDF Power", power);
        telemetryM.addData("PIDF Error", spindexerPidf.getPositionError());
        telemetryM.addData("Absolute Position", this.getCurrentAbsolutePosition());

    }
    //==================================================================================================
    //  I N I T I A L I Z A T I O N   &   S T A T E   M A C H I N E
    //==================================================================================================

    /**
     * Executes the stateful homing sequence to find the zero position + offset
     * This method must be called repeatedly in a loop (e.g., in an OpMode's init_loop).
     *
     * @return True when the entire homing process is complete; otherwise false.
     */
    public boolean doInitPosition(boolean force) {

        // If a state has been caried over use that
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

        // start a rotate to slot 0
        if (homingState.equals(HomingState.START)) {
            octoquad.resetSinglePosition(REV_PWM_1);
            // force the octoquad to reset the wrap value
            octoquad.setSingleChannelPulseWidthTracksWrap(REV_PWM_1, false);
            octoquad.setSingleChannelPulseWidthTracksWrap(REV_PWM_1, true);
            this.curShootSlot = 0;
            int currentPosition = getCurrentPosition();
            double shortestPathDelta = calculateShortestPathDelta(currentPosition, INDEX_OFFSET_TICKS, TICKS_PER_REV);
            this.lastTargetPosition = (int) Math.round(currentPosition + shortestPathDelta);
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

    /**
     * Calculates the shortest delta in ticks to reach a target position from the current position,
     * accounting for wrapping around the spindexer's full rotation.
     *
     * @param currentPosition    The current raw encoder position of the motor.
     * @param targetPosition     The absolute target encoder position.
     * @param ticksPerRevolution The total number of encoder ticks for one full revolution (e.g., NUMBER_OF_SLOTS * TICKS_PER_SLOT).
     * @return The shortest delta in ticks to apply to the current position to reach the target.
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
        if (Math.abs(directDelta) <= Math.abs(wrappedDelta)) {
            return directDelta;
        } else {
            return wrappedDelta;
        }
    }


    //==================================================================================================
    //  C O R E   M O V E M E N T   M E T H O D S
    //==================================================================================================

    public int getCurrentPosition() {
        OctoQuad.EncoderDataBlock dataBlock = octoquad.readAllEncoderData();
        return dataBlock.positions[REV_PWM_1];
    }

    public int getCurrentAbsolutePosition() {
        return Math.floorMod(getCurrentPosition(), REV_PWM_HIGH) + 1;
    }

    public double getCurrentAbsoluteAngle() {
        return (getCurrentAbsolutePosition() * DEGREES_PER_US);
    }

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
        curShootSlot = slotNumber;
    }

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


//==================================================================================================
//  S L O T   M A N A G E M E N T
//==================================================================================================

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
        int finalTargetPosition = lastTargetPosition + SLOT_OFFSET_TICKS[curShootSlot];

        // Calculate the error: the difference between where we want to be and where we are.
        double error = finalTargetPosition - this.getCurrentPosition();

        // Check all conditions for readiness:
        //    - The position error is within the acceptable tolerance.
        //    - The minimum time for rotation has passed (to prevent premature triggers).
        return Math.abs(error) <= POSITION_TOLERANCE
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
     * Checks if all slots are full.
     *
     * @return true for full otherwise false
     */
    public boolean isFull() {
        return Arrays.stream(slotStates).noneMatch(state -> state == SlotState.EMPTY);
    }

    /**
     * Checks if all slots are empty.
     *
     * @return true for empty otherwise false
     */
    public boolean isEmpty() {
        return Arrays.stream(slotStates).allMatch(state -> state == SlotState.EMPTY);
    }
}