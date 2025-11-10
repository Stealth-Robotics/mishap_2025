package org.firstinspires.ftc.teamcode.systems;

import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.Map;

/**
 * Container of all subsystems for the robot and wrapper for all controls.
 * This class manages the robot's overall state and coordinates actions between various subsystems
 * like the intake, shooter, and drivetrain. It uses a state machine to handle timed,
 * sequential operations, ensuring actions occur in the correct order.
 *
 * @author Team Mishap
 * @version 1.0
 */
@Configurable
public class RobotSystem {

    //==================================================================================================
    // Constants
    //==================================================================================================

    /** Multiplier to reduce drive speed for fine-tuned control. */
    public static double SLOW_MODE_MULTIPLIER = 0.3;

    public static double AUTO_AIM_TOLERANCE = 0.3;


    /** The maximum rotational power applied during auto-aim. */
    public static final double MAX_ROTATION_POWER = 0.90;

    public static final double MIN_ROTATION_POWER = .07;

    public static final double ROTATION_SIGNUM_POWER = .06;


    public static final int MAX_OVER_CURRENT_COUNT = 4;

    private static final double MAX_UNJAM_TIME = 1000;

    /** PIDF coefficients for the heading controller used in auto-aim. */
    private static final PIDFCoefficients HEADING_COEFFICIENTS
            = new PIDFCoefficients(0.016, 0, 0, 0); //0.018, 0.0001, 0.001, 0.02

    /** Maps the current motif pattern to a list of slot states. */
    private static final Map<Motif, List<SlotState>> motifSlots = Map.of(
            Motif.GPP, List.of(SlotState.ARTIFACT_GREEN, SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_PURPLE),
            Motif.PGP, List.of(SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_GREEN, SlotState.ARTIFACT_PURPLE),
            Motif.PPG, List.of(SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_GREEN),
            Motif.UNKNOWN, List.of(SlotState.UNKNOWN, SlotState.UNKNOWN, SlotState.UNKNOWN)
    );

    public boolean isSpindexerEmpty() {
        return spindexerSys.isEmpty();
    }

    //==================================================================================================
    // State Machine
    //==================================================================================================

    /**
     * Defines the possible operational states of the RobotSystem.
     */
    private enum SystemState {
        IDLE,               // Default state, robot is waiting for commands.
        INTAKING,           // Actively running the intake and spindexer.
        REVERSING_INTAKE,   // Reversing the intake to clear jams.
        STOPPING_INTAKE,    // A transitional state to properly stop the intake sequence.
        PREPPING_SHOOT,     // A transitional state to prepare the shooter for shooting.
        SHOOT_IT,     // Spinning up the shooter and positioning the hood.
        AFTER_SHOT,            // A transitional state to shoot one artifact and advance the spindexer.
        SPINDEXING           // A requst to move the spindexer
    }

    private SystemState currentState = SystemState.IDLE;
    //private final Deque<SystemState> commandQueue = new LinkedList<>();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime jamTimer = new ElapsedTime();


    //==================================================================================================
    // Member Variables
    //==================================================================================================

    // --- Subsystems ---
    private final SweeperSubsystem sweeperSys;
    private final HoodSubsystem hoodSys;
    private final KickerSubsystem kickerSys;
    private final ShooterSubsystem shooterSys;
    private final SpindexerSubsystem spindexerSys;
    private final LimelightSubsystem limelightSys;
    private final Follower follower;
    private final ColorSensorSubsystem colorSensorSys;

    // --- Control and Telemetry ---
    private static final PIDFController headingController = new PIDFController(HEADING_COEFFICIENTS);
    private final TelemetryManager telemetryM;
    private final Telemetry telemetry;

    private volatile boolean isShootReady = false;

    /** The current motif pattern being used. */
    private Motif motifPattern = Motif.UNKNOWN;

    /** Current index into the motif patter */
    private int curMotifIndex = 0;

    /** This keeps the shooter motors spinning */
    private boolean isBurstFire = false;

    private boolean isMotifShot = false;

    private boolean isMotifAvailable = false;


    /** tracks if the robot will handle intaking */
    private boolean isAutoIntaking = true;

    /** Number of times in a row we've hit over current*/
    private int overCurrentCount = 0;

    private double controlInverter = 1;

    private boolean wasAutoAim = false;

    //==================================================================================================
    // Constructor
    //==================================================================================================

    /**
     * Constructs a new RobotSystem and initializes all its subsystems.
     *
     * @param hardwareMap The robot's configured HardwareMap.
     * @param telemetry   The OpMode's Telemetry object for displaying information.
     */
    public RobotSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sweeperSys = new SweeperSubsystem(hardwareMap);
        this.hoodSys = new HoodSubsystem(hardwareMap);
        this.kickerSys = new KickerSubsystem(hardwareMap);
        this.shooterSys = new ShooterSubsystem(hardwareMap);
        this.spindexerSys = new SpindexerSubsystem(hardwareMap);
        this.limelightSys = new LimelightSubsystem(hardwareMap);
        this.follower = Constants.createFollower(hardwareMap);
        this.colorSensorSys = new ColorSensorSubsystem(hardwareMap, telemetry);

        Drawing.init();
        this.telemetry = telemetry;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    //==================================================================================================
    // Initialization
    //==================================================================================================

    /**
     * Initializes the spindexer by rotating it until its index sensor is triggered,
     * then centers it on a known slot. This should be called once during initialization.
     *
     * @return true when the spindexer is successfully indexed and centered, otherwise false.
     */
    public boolean doInitSpindexer() {
        if (!hoodSys.isReadyToShoot()
                || !kickerSys.isReady()
                || currentState != SystemState.IDLE) {
            return false;
        }

        return spindexerSys.doInitPosition();
    }



    //==================================================================================================
    // Core Logic
    //==================================================================================================

    /**
     * Main update loop for the RobotSystem. This method should be called repeatedly in the main
     * loop of an OpMode. It manages subsystem updates, state transitions, and telemetry.
     */
    public void update() {
        // Always update critical subsystems
        follower.update();
        limelightSys.update();
        shooterSys.update();
        colorSensorSys.update();

        // incase we getting spindexer jamming
        handleOverCurrent();

        // Update the overall shoot readiness flag based on subsystem states.
        this.isShootReady =
                 shooterSys.isReadyToShoot()
                && hoodSys.isReadyToShoot()
                && kickerSys.isReady()
                && spindexerSys.isReady();

        // Process the state machine logic for timed actions.
        handleStateTransitions();

        // Update telemetry displayed on the driver station.
        displayTelemetry();
    }

    /**
     * Protects the robot from a jammed spindexer
     */
    private void handleOverCurrent() {
        // Exit early if the motor is not jammed. Reset the counter.
        if (!spindexerSys.isJammed()) {
            if (overCurrentCount > 0) {
                if (jamTimer.milliseconds() > MAX_UNJAM_TIME) {
                    if (currentState == SystemState.INTAKING) {
                        stopIntake();
                    }
                    // If jam free for over MAX_UNJAM_TIME then reset the counter
                    overCurrentCount = 0;
                }
            }

            return;
        }

        // Increment the counter to track that a jam is ongoing.
        overCurrentCount++;

        // Check if the robot should be completely stopped due to a persistent jam.
        if (overCurrentCount >= MAX_OVER_CURRENT_COUNT) {
            spindexerSys.setEmergencyStop();
            sweeperSys.stopIntake();
            return;
        }

        // Only execute the anti-jam maneuver if enough time has passed.
        if (jamTimer.milliseconds() > MAX_UNJAM_TIME) {
            // The logic inside this block is your anti-jam maneuver.
            // It toggles between two actions based on whether the number of attempts is odd or even.

            if ((overCurrentCount & 1) == 1) {
                // ODD attempt (1st, 3rd, etc.): Reverse the intake and spindexer.
                spindexerSys.setFloat();      // Allow spindexer to move freely
                sweeperSys.reverseIntake();   // Reverse the sweeper
                hoodSys.setIntakePose();
                spindexerSys.decreaseOneSlot();       // Actively reverse the spindexer
                spindexerSys.setBrake();
            } else {
                this.isAutoIntaking = false;
                this.stopIntake();
            }

            // Reset the timer AFTER executing the maneuver to wait for the next cycle.
            jamTimer.reset();
        }
    }

    /**
     * Controls the robot's drivetrain movement and wraps the follower's tele-op drive logic.
     *
     * @param y            Forward/backward power (-1 to 1).
     * @param x            Strafe left/right power (-1 to 1).
     * @param z            Rotational power (-1 to 1).
     * @param robotCentric If true, drive directions are relative to the robot's front. If false, they are field-centric.
     * @param slowMo       If true, applies the SLOW_MODE_MULTIPLIER to reduce drive speed.
     * @param isAutoAim      If true, uses the Limelight camera to automatically aim at a target, overriding the z input.
     */
    public void drive(double y, double x, double z, boolean robotCentric, boolean slowMo, boolean isAutoAim) {
        double slowMoFactor = slowMo ? SLOW_MODE_MULTIPLIER : 1;
        double turn = z * slowMoFactor;

        if (isAutoAim) {
            wasAutoAim = true;
            limelightSys.getLastResult(); // Force an update of the Limelight data.
            Pose llPose = limelightSys.getAvgTargetPose(50); // Get averaged target position.
            if (llPose != null) {
                // Calculate the turn power needed to center the target.
                double distance = LimelightSubsystem.calcGoalDistanceByTy(llPose.getY());
                shooterSys.setTargetRpmFromDisance(distance);
                spindexerSys.setOffsetByDistance(distance);
                //telemetryM.addData("Power OUT:", output);
                //telemetryM.addData("LimeLightTX:", llPose.getX());
                // The output is applied to the rotation power (note: may need to be inverted).
                turn = getScaledTxOutput(llPose.getX(), AUTO_AIM_TOLERANCE);

            } else {
                // If the target is lost, reset the PID controller to prevent integral windup.
                headingController.reset();
            }
        } else if (wasAutoAim) {
            headingController.reset();
            wasAutoAim = false;
        }

        //telemetryM.addData("TURN:", turn);
        // Set drive power to the follower.
        follower.setTeleOpDrive(
                y * slowMoFactor * controlInverter,
                x * slowMoFactor * controlInverter,
                turn,
                robotCentric);

        // It's recommended to call update after setting power for immediate effect.
        follower.update();
    }

    /**
     * this is is for autonomous mode and is designed to be called in a loop until done
     * @param tolerance tolerance in degrees
     * @param latency number of milliseconds to sample the average pose
     * @return true if done aiming otherwise false
     */
    public boolean doAimAtTarget(double tolerance, double offset, long latency) {
        if (!follower.isTeleopDrive()) {
            follower.startTeleopDrive(true);
        }

        limelightSys.getLastResult(); // Force an update of the Limelight data.
        Pose llPose = limelightSys.getAvgTargetPose(latency); // Get averaged target position.
        double turn = 0;
        if (llPose != null) {
            // Calculate the turn power needed to center the target.
            double output = getScaledTxOutput(llPose.getX() + offset, tolerance);
            double distance = LimelightSubsystem.calcGoalDistanceByTy(llPose.getY());
            shooterSys.setTargetRpmFromDisance(distance);
            spindexerSys.setOffsetByDistance(distance);
            // The output is applied to the rotation power (note: may need to be inverted).
            turn = output;
        } else {
            // If the target is lost, reset the PID controller to prevent integral windup.
            headingController.reset();
        }

        follower.setTeleOpDrive(0, 0, turn, true);
        return turn == 0 && llPose != null;
    }

    /**
     * Reads the current motif pattern from the Limelight camera.
     * This method is non-blocking and can be called repeatedly.
     *
     * @return true if a valid motif pattern has been successfully read, otherwise false.
     */
    public boolean doReadMotif() {
        if (limelightSys.getCurrentPipeline() != Pipeline.APRILTAG_MOTIF) {
            if (limelightSys.setPipeline(Pipeline.APRILTAG_MOTIF)) {
                // Waiting for the pipeline to switch, not yet ready to read.
                return false;
            }
        }

        AprilTagIds id = limelightSys.getAprilTagId();
        Motif detectedMotif = Motif.fromId(id.id);

        if (detectedMotif != Motif.UNKNOWN) {
            // A valid motif (GPP, PGP, or PPG) was found.
            this.motifPattern = detectedMotif;
            Motif.set(detectedMotif);
            return true;
        } else if (id.id == AprilTagIds.TAG_ID_NONE.id) {
            // No tag is visible, update state to UNKNOWN.
            this.motifPattern = Motif.UNKNOWN;
        }

        // Return false if the pipeline is still switching, no tag is seen,
        // or a non-motif tag is visible.
        return false;
    }

    public Motif getDetectedMotif() {
        return this.motifPattern;
    }


    //==================================================================================================
    // State Control Methods
    //==================================================================================================

    public void setAutoIntaking(boolean autoIntaking) {
        isAutoIntaking = autoIntaking;
    }

    public void toggleAutoIntaking() {
        isAutoIntaking = !isAutoIntaking;
    }

    /**
     * Starts the intake process. Stops the shooter, sets hood to intake position,
     * and runs the sweeper and spindexer.
     */
    public void startIntake() {
        if (!isBurstFire) {
            shooterSys.stop();
        }

        kickerSys.setReady();
        if (spindexerSys.getIntakeSlotState() == SlotState.EMPTY) {
            hoodSys.setIntakePose();
            sweeperSys.startIntake();
            spindexerSys.setFloat();
            currentState = SystemState.INTAKING;
        }
        else if (spindexerSys.getShootSlotState() == SlotState.EMPTY ) {
            incrementSpindexerSlot();
        }
        else if (!isSpindexerFull()){
            decrementSpindexerSlot();
        }
    }

    /**
     * Reverses the intake and spindexer motors to clear jams.
     */
    public void reverseIntake() {
        currentState = SystemState.REVERSING_INTAKE;
        kickerSys.isReady();
        hoodSys.setIntakePose();
        sweeperSys.reverseIntake();
        spindexerSys.setFloat();
    }

    /**
     * Initiates the sequence to stop the intake process. This is a timed action
     * that will transition the state to IDLE after a short delay.
     */
    public void stopIntake() {
        // Only proceed if we are currently intaking or reversing.
        if (currentState == SystemState.INTAKING || currentState == SystemState.REVERSING_INTAKE) {
            currentState = SystemState.STOPPING_INTAKE;
            stateTimer.reset();
            spindexerSys.setFloat();
            hoodSys.setShootPose(); // Start moving hood to shoot position immediately.
        }
    }

    public void toggleIntake() {
        if (currentState == SystemState.INTAKING
                || currentState == SystemState.REVERSING_INTAKE) {
            stopIntake();
        } else {
            startIntake();
        }
    }

    /**
     * Prepares the robot to shoot by spinning up the shooter wheels and setting the hood angle.
     */
    public void setReadyShoot() {
        if (currentState != SystemState.PREPPING_SHOOT
            && currentState != SystemState.STOPPING_INTAKE) {

            if (currentState == SystemState.REVERSING_INTAKE
                    || currentState == SystemState.INTAKING) {
                this.stopIntake();
                return;
            }

            shooterSys.runShooter();
            // Important we set this after stop intake
            currentState = SystemState.PREPPING_SHOOT;
        }
    }

    /**
     * Executes a shot if all subsystems are ready. Kicks a projectile and advances the spindexer.
     * This is a timed action that will transition the state to IDLE after completion.
     *
     * @return true if the shot was successfully initiated, false otherwise.
     */
    public boolean tryShoot() {
        // Only allow a shot if we are in the PREPPING_SHOOT state
        // and the shooter subsystem reports it is at the correct speed.
        if (currentState == SystemState.PREPPING_SHOOT) {
            stateTimer.reset();
            currentState = SystemState.SHOOT_IT;
            return true;
        } else if (currentState == SystemState.IDLE) {
            setReadyShoot();
        }

        // If we aren't ready to shoot, return false.
        return false;
    }

    /**
     * Resets the Motif state machine back to the initial state.
     */
    public void resetMotifState() {
        isMotifShot = false;
        curMotifIndex = 0;
        setBurstFire(false);
        stopShooter();
    }
    /**
     * Attempts to get ready for a motif shot
     * @return returns false until the robot can be readied or the spindexer is empty
     */
    public boolean startShootMotif(boolean useBurstFire) {
        // Spindexer empty no need to continue
        if (spindexerSys.isEmpty()) {
            resetMotifState();
            return false;
        }

        isMotifAvailable = spindexerSys.isMotifAvailable();

        // keep the shooter spinning
        setBurstFire(useBurstFire);
        isMotifShot = true;
        curMotifIndex = 0;

        // start preping our shots
        setReadyShoot();

        return true;
    }

    /**
     * Call after startShootMotif() to continue shooting the motif
     *
     * @return returns false until done shooting the motif
     */
    public boolean continueShootMotif() {
        telemetry.addData("Motif index", curMotifIndex);
        // must wait for the spindexer or shot cycle to finish
        if (this.isSpindexerBusy() || currentState == SystemState.SHOOT_IT) {
            return false;
        }

        // Can't shoot anymore
        if (spindexerSys.isEmpty() && curMotifIndex >= 3) {
            resetMotifState();
            return true;
        }

        // If we don't have a Motif then just shoot it all
        if (!isMotifAvailable) {
            if (spindexerSys.getShootSlotState() == SlotState.EMPTY) {
                if (!spindexerSys.moveToNextAvailableSlot())
                {
                    // unable to move to the next slot
                    resetMotifState();
                    return !this.isSpindexerBusy();
                }

                return false; // continue
            }

            // this will either prep for shooting or shoot the current
            tryShoot();

            return false;
        }

        //noinspection DataFlowIssue
        if (curMotifIndex >= motifSlots.get(motifPattern).size()) {
            resetMotifState();
            return this.isSpindexerBusy();
        }

        // OK we can shoot the motif

        //noinspection DataFlowIssue
        SlotState slotState = motifSlots.get(motifPattern).get(curMotifIndex);
        telemetryM.addData("Motif State", slotState);
        // if we have the correct arficat in the shoot slot we can go to shooting
        if (spindexerSys.getShootSlotState() != slotState) {
            // if error in the rotate logic stop
            if(spindexerSys.rotateToArtifact(slotState) || this.isSpindexerBusy()){
                return false;
            } else if (!spindexerSys.moveToNextAvailableSlot()) {
                resetMotifState();
                return true;
            }
        }

        tryShoot();
        return false;
    }

    /**
     * Blindly attempts to move the spindexer to the first available Motif slot.
     */
    public void trySelectFirstMotifSlot() {
        curMotifIndex = 0;
        SlotState desiredArtifact = motifSlots.get(motifPattern).get(curMotifIndex);
        if (!spindexerSys.rotateToArtifact(desiredArtifact)){
            spindexerSys.advanceOneSlot();
        }
    }

    public boolean isAnyArtifactUnknown() {
        for (SlotState slotState : motifSlots.get(motifPattern)) {
            if (slotState == SlotState.UNKNOWN) {
                return true;
            }
        }

        return false;
    }

    //==================================================================================================
    // Getters and Status Checks
    //==================================================================================================

    /**
     * Checks if the intake system is currently running (either forward or in reverse).
     *
     * @return true if the robot state is INTAKING or REVERSING_INTAKE.
     */
    public boolean isSpindexerBusy() {
        return currentState == SystemState.INTAKING
                || currentState == SystemState.REVERSING_INTAKE
                || currentState == SystemState.STOPPING_INTAKE
                || !spindexerSys.isReady()
                || !kickerSys.isReady();
    }

    /**
     * Returns whether the robot is ready to shoot.
     *
     * @return The value of the volatile isShootReady flag.
     */
    public boolean getShootReady() {
        return this.isShootReady;
    }

    public boolean isHoodShootPose() {
        return hoodSys.isReadyToShoot();
    }

    /**
     * Gets the current Pedro pose from limelight 3D pose
     * @param latency number of milliseconds to sample the average pose
     * @return the current pose of the robot or null if no pose is available
     */
    @Nullable
    public Pose getPedroPoseFromLimelight(long latency)
    {
        Pose3D botPose = limelightSys.getAvgBotPose(latency);
        if (botPose == null) {
            return null;
        }

        return LimelightSubsystem.limelightToPedroPose(botPose);
    }

    //==================================================================================================
    // Passthrough and Utility Methods
    //==================================================================================================

    public void stopShooter() {
        if (shooterSys.isRunning()){
            kickerSys.setReady();
            shooterSys.stop();
        }
    }

    public void startShooter() {
        if(!shooterSys.isRunning()){
            shooterSys.runShooter();
        }
    }

    private boolean isSortingArtifacts = false;

    public boolean doArtifactSort() {
        if (!hoodSys.isReadyToShoot()) {
            hoodSys.setShootPose();
            return false;
        }

        if (this.isSpindexerBusy()) {
            return false;
        }

        if (!isSortingArtifacts){
            isSortingArtifacts = true;
            // Assume the spindexer is empty to shortcut a spin cycle
            if (spindexerSys.getIntakeSlotState() == SlotState.EMPTY) {
                return true;
            }
        }

        return spindexerSys.doCheckForArtifacts();
    }

    /**
     * Gets the path-following controller instance.
     *
     * @return The {@link Follower} object used for pathing and driving.
     */
    public Follower getFollower() {
        return follower;
    }

    // TODO: Temp Code:
    public void resetHighLowColors() {
        colorSensorSys.resetHighLow();
    }

    public boolean getColorSensorDetectedArtifact() {
        return colorSensorSys.getLastDetection() != SlotState.EMPTY;
    }

    /** SetSlotsEmpty */
    public void initSpindxerSlotsEmpty() {
        spindexerSys.setSlotsEmpty();
    }

    /**
     * Sets the spindexer to a GPP state to start the auto
     */
    public void initSpindxerSlotsAuto() {
        spindexerSys.setSlotsAuto();
    }

    /**
     * Checks if the spindexer is full.
     * @return true if the spindexer is full, false otherwise
     */
    public boolean isSpindexerFull() {
        return spindexerSys.isFull()
                && hoodSys.isReadyToShoot();
    }

    /** Sets the Obelisk motif pattern. */
    public void setMotifPattern(Motif motifPattern) { this.motifPattern = motifPattern; }

    /** Manually advances the spindexer by one slot. */
    public void incrementSpindexerSlot() {
        if (kickerSys.isReady()) {
            spindexerSys.advanceOneSlot();
        }
    }

    /** Manually moves the spindexer back by one slot. */
    public void decrementSpindexerSlot() { spindexerSys.decreaseOneSlot(); }

    /** Rotates the spindexer to a specific slot index. */
    public void setSpindexerSlot(int index) { spindexerSys.rotateToSlot(index); }

    /** Increases the target shooter RPM for far shots. */
    public void increaseShooterRpm() { shooterSys.increaseCurrentRpmRange(); }

    /** Decreases the target shooter RPM for far shots. */
    public void decreaseShooterRpm() { shooterSys.decreaseCurrentRpmRange(); }

    /** Nudges the spindexer's raw position by a small positive amount. */
    public void increaseSpindexer() { spindexerSys.nudgePosition(5); }

    /** Nudges the spindexer's raw position by a small negative amount. */
    public void decreaseSpindexer() { spindexerSys.nudgePosition(-5); }

    /** Sets the shooter's target range */
    public void setShooterTargetRangeNear() { shooterSys.setTargetRange(ZoneDistance.NEAR); }
    public void setShooterTargetRangeFar() { shooterSys.setTargetRange(ZoneDistance.FAR); }
    public void setShooterTargetRangeMid() { shooterSys.setTargetRange(ZoneDistance.MID); }

    /** Toggles the Limelight's active pipeline. */
    public void toggleLimelightTarget() { limelightSys.toggleTargetPileline(); }

    public void toggleFullLimelightPipeline(){ limelightSys.toggleFullPipeline(); }

    // ** Sets the Limelight's active pipeline. */
    public void setLimelightPipeline(Pipeline pipeline) { limelightSys.setPipeline(pipeline); }

    public Pipeline getLimelightPipeline() { return limelightSys.getCurrentPipeline(); }


    public void setControlsInverted() {
        this.controlInverter = -1;
    }


    /**
     * Resets the IMU's heading. Catches and logs any interruption.
     */
    public void resetIMU() {
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
            this.controlInverter = 1;
            //follower.getPoseTracker().resetHeadingToIMU();
            follower.setPose(new Pose(72,72, 0));
        } catch (InterruptedException ie) {
            telemetryM.debug("IMU reset failed");
        }
    }

    /**
     * Sets the burst fire mode. If disabled, it also stops the shooter.
     * @param burstFire true to enable burst fire, false to disable.
     */
    public void setBurstFire(boolean burstFire) {
        this.isBurstFire = burstFire;
        if (!burstFire) {
            shooterSys.stop();
        }
    }

    //==================================================================================================
    // Private Helper Methods
    //==================================================================================================

    /**
     * Manages timed state transitions using an ElapsedTime timer. This is called
     * by the main update() loop.
     */
    private void handleStateTransitions() {
        switch (currentState) {
            case STOPPING_INTAKE:
                // The hood will let us know when it is closed check the delay setting in HoodSubsystem
                if (hoodSys.isReadyToShoot()) {
                    sweeperSys.stopIntake();
                    spindexerSys.setBrake();

                    SlotState slotState = colorSensorSys.getLastDetection();
                    if (slotState != SlotState.EMPTY) {
                        spindexerSys.setIntakeSlotState(slotState);
                        if (spindexerSys.getShootSlotState() == SlotState.EMPTY) {
                            spindexerSys.advanceOneSlot();
                            currentState = SystemState.SPINDEXING;
                            break;
                        }
                    }else {
                        spindexerSys.setIntakeSlotEmpty();
                    }

                    currentState = SystemState.IDLE;
                }
                break;

            case SHOOT_IT:
                if (isShootReady)
                {
                    kickerSys.kickIt();
                    shooterSys.resetRpmCounter();
                    spindexerSys.setShootSlotEmpty();
                    currentState = SystemState.AFTER_SHOT;
                    stateTimer.reset();
                }
                break;

            case AFTER_SHOT:

                // hold the shooter for a short delay to allow the kicker to kick the ball
                if (stateTimer.milliseconds() >= KickerSubsystem.KICK_DELAY) {
                    kickerSys.setReady();
                    // Stop the shooter unless burst fire is active.
                    if (!isBurstFire) {
                        shooterSys.stop();
                    }
                    // We will auto handle the rotation
                    if (isMotifShot && kickerSys.isReady()) {
                        curMotifIndex++;
                        currentState = SystemState.IDLE;
                    } else if (kickerSys.isReady()) {
                        spindexerSys.advanceOneSlot();
                        currentState = SystemState.SPINDEXING;
                    }
                }

                break;
                case SPINDEXING:
                    if (spindexerSys.isReady()){
                        currentState = SystemState.IDLE;
                    }

                    break;
            case IDLE:
                // IDLE state keep an eye on the intaking slot
                // if there is a change update the state
                SlotState detectedState = colorSensorSys.getLastDetection();
                if (detectedState != SlotState.TRANSITIONING
                        && hoodSys.isReadyToShoot()
                        && spindexerSys.isReady()) {
                    SlotState currentSlotState = spindexerSys.getIntakeSlotState();

                    // Shooting should empty slots so we just want to add here
                    if (detectedState != SlotState.EMPTY) {
                        if (currentSlotState == SlotState.EMPTY
                                || currentSlotState == SlotState.UNKNOWN) {
//                    if ((detectedState != SlotState.UNKNOWN
//                            && detectedState != currentSlotState)
//                            || (detectedState == SlotState.UNKNOWN
//                            && currentSlotState == SlotState.EMPTY)) {
                            spindexerSys.setIntakeSlotState(detectedState);
                        }
                    }
                }

                break;
            case INTAKING:
                if (isAutoIntaking) {
                    // Just incase make sure we move the kicker
                    kickerSys.setReady();

                    SlotState itemDetected = colorSensorSys.getLastDetection();

                    // if the spindexer is in float it can't be in the ready state
                    spindexerSys.setBrake();

                    // Condition 1: An artifact is detected.
                    if (itemDetected != SlotState.EMPTY
                            && hoodSys.isReadyInTake()
                            && spindexerSys.isReady()
                            && kickerSys.isReady()) {

                        // First, check if we need to stop because the shooter slot is full.
                        if (spindexerSys.getShootSlotState() != SlotState.EMPTY) {
                            // This will close the hood and handle the state transition correctly.
                            this.stopIntake();
                            break; // Exit the INTAKING case
                        }

                        // Condition 2: The spindexer is ready to move.
                        if (spindexerSys.isReady() && kickerSys.isReady()) {

                            // If the shooter slot is empty, proceed to advance the spindexer.
                            spindexerSys.setIntakeSlotState(itemDetected);
                            spindexerSys.advanceOneSlot();
                            // By advancing, spindexerSys.isReady() will become false until the move is complete,
                            // preventing this block from running again for the same artifact.
                        }
                    }
                }
                break;
            case REVERSING_INTAKE:
                break;
        }
    }

    /**
     * Calculates the required rotational power to correct a given heading error (txDelta)
     * using a PIDF controller. The output is clamped to a maximum power.
     *
     * @param txDelta   The horizontal error from the target (e.g., Limelight's 'tx').
     * @param tolerance The error margin within which no correction is applied.
     * @return The calculated rotational power, or 0 if within tolerance.
     */
    private double getScaledTxOutput(double txDelta, double tolerance) {

        if (Math.abs(txDelta) < tolerance) {
            headingController.reset();
            return 0;
        }

        // Set the PID controller's target to 0 (no error).
        headingController.setTargetPosition(0);
        // Update the controller with the current error.
        headingController.updateError(-txDelta);
        // Run the PID calculation.
        double pidOutput = headingController.run();

        // If the error is outside the tolerance, return the clamped PID output.
        // Otherwise, return 0 to stop turning.
        if (Math.abs(pidOutput) > 1e-4 && Math.abs(pidOutput) < MIN_ROTATION_POWER) {
            pidOutput = pidOutput + (Math.signum(pidOutput) * ROTATION_SIGNUM_POWER);
            if (Math.abs(pidOutput) > MIN_ROTATION_POWER){
                pidOutput = Math.signum(pidOutput) * MIN_ROTATION_POWER;
            }else
            if (Math.abs(pidOutput) < MIN_ROTATION_POWER) {
                pidOutput = 0;
            }
        }

        return  MathFunctions.clamp(pidOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
    }

    /**
     * Displays relevant robot data on the driver station via TelemetryManager.
     */
    private void displayTelemetry() {
        // TODO: remove unneeded output
        telemetryM.addData("Shooter RPM (avg)", shooterSys.getCurrentRpm());
        telemetryM.addData("Target RPM", shooterSys.getTargetRpm());
//        telemetryM.addData("Is Auto Intaking:", isAutoIntaking);
//        telemetryM.addData("Is Burst MODE", isBurstFire);
        telemetryM.addData("Spindexer offset:", spindexerSys.getCurrentOffset());
        telemetryM.addData("Is Spindexer Ready", spindexerSys.isReady());
//        telemetryM.addData("Current zone", spindexerSys.getCurrentZone());
//        telemetryM.addData("Robot State", currentState.name());
//        telemetryM.addData("Shoot Slot Index", spindexerSys.getCurShootSlot());
        telemetryM.addData("Shoot Slot State:", spindexerSys.getShootSlotState());
        telemetryM.addData("Standbby Slot State:", spindexerSys.getStandbySlotState());
        telemetryM.addData("Intake Slot State:", spindexerSys.getIntakeSlotState());
//        telemetryM.addData("Shoot Slot Number:", spindexerSys.getCurShootSlot());
    //   telemetryM.addData("IsMotif available", spindexerSys.isMotifAvailable());
//        telemetryM.addData("Hood State:", hoodSys.getHoodState());
//        telemetryM.addData("Motor RPM:", shooterSys.getMotorRpms());
//        telemetryM.addData("shooterSys Ready:", shooterSys.isReadyToShoot());
//        telemetryM.addData("isShootReady:", isShootReady);
//        telemetryM.addData("Spindexer Raw Position", spindexerSys.getCurrentPosition());
//        telemetryM.addData("CURRENT HEADING", follower.getHeading());

        // TODO: THIS SHOULD BE REMOVED BEFORE COMP
        this.draw();
        /* INPORTAINT This updates the telemetry for all systems here no need to duplicate anywhere else */
        telemetryM.update(telemetry);
    }
    private void drawOnlyCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    private void draw() {
        Drawing.drawDebug(follower);
    }

}