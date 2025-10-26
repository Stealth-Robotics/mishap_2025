package org.firstinspires.ftc.teamcode.systems;

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
import org.firstinspires.ftc.teamcode.common.AprilTagIds;
import org.firstinspires.ftc.teamcode.common.Motif;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.common.SlotState;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Dictionary;
import java.util.Hashtable;
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
    public static final double MAX_ROTATION_POWER = 0.80;

    public static final int MAX_OVER_CURRENT_COUNT = 4;


    /** PIDF coefficients for the heading controller used in auto-aim. */
    private static final PIDFCoefficients HEADING_COEFFICIENTS
            = new PIDFCoefficients(0.018, 0.0, 0.001, 0.02);

    /** Delay in milliseconds to wait before resetting the kicker to its ready position after shooting. */
    private static final long SHOOT_KICKER_RESET_DELAY_MS = 200;

    /** Delay in milliseconds to wait before advancing the spindexer to the next slot after shooting. */
    private static final long SHOOT_SPINDEXER_ADVANCE_DELAY_MS = SHOOT_KICKER_RESET_DELAY_MS * 2;

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
        PREPPING_SHOOT,     // Spinning up the shooter and positioning the hood.
        SHOOTING,            // A transitional state to fire a projectile and advance the spindexer.
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

    /** Maps the current motif pattern to a list of slot states. */
    private final Map<Motif, List<SlotState>> motifSlots = Map.of(
            Motif.GPP, List.of(SlotState.ARTIFACT_GREEN, SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_PURPLE),
            Motif.PGP, List.of(SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_GREEN, SlotState.ARTIFACT_PURPLE),
            Motif.PPG, List.of(SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_PURPLE, SlotState.ARTIFACT_GREEN),
            Motif.UNKNOWN, List.of(SlotState.UNKNOWN, SlotState.UNKNOWN, SlotState.UNKNOWN)
    );

    /** This keeps the shooter motors spinning */
    private boolean isBurstFire = false;

    /** tracks if the robot will handle intaking */
    private boolean isAutoIntaking = false;

    /** Number of times in a row we've hit over current*/
    private int overCurrentCount = 0;

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
        this.isShootReady = (currentState == SystemState.IDLE || currentState == SystemState.PREPPING_SHOOT)
                && shooterSys.isReadyToShoot()
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
            if (overCurrentCount > 0){

                if (jamTimer.milliseconds() > 1000){
                    if (currentState == SystemState.INTAKING) {
                        sweeperSys.startIntake();
                    }
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
        if (jamTimer.milliseconds() > 1000) {
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
     * @param autoAim      If true, uses the Limelight camera to automatically aim at a target, overriding the z input.
     */
    public void drive(double y, double x, double z, boolean robotCentric, boolean slowMo, boolean autoAim) {
        double slowMoFactor = slowMo ? SLOW_MODE_MULTIPLIER : 1;
        double turn = z * slowMoFactor;

        if (autoAim) {
            limelightSys.getLastResult(); // Force an update of the Limelight data.
            Pose llPose = limelightSys.getAvgTargetPose(10); // Get averaged target position.

            if (llPose != null) {
                // Calculate the turn power needed to center the target.
                double output = getScaledTxOutput(llPose.getX(), AUTO_AIM_TOLERANCE);
                // The output is applied to the rotation power (note: may need to be inverted).
                turn = -output;
            } else {
                // If the target is lost, reset the PID controller to prevent integral windup.
                headingController.reset();
            }
        }

        // Set drive power to the follower.
        follower.setTeleOpDrive(y * slowMoFactor, x * slowMoFactor, turn, robotCentric);
        // It's recommended to call update after setting power for immediate effect.
        follower.update();
    }

    public boolean doAimAtTarget(double tolerance, long latency) {
        limelightSys.getLastResult(); // Force an update of the Limelight data.
        Pose llPose = limelightSys.getAvgTargetPose(latency); // Get averaged target position.
        double turn = 0;
        if (llPose != null) {
            // Calculate the turn power needed to center the target.
            double output = getScaledTxOutput(llPose.getX(), tolerance);
            // The output is applied to the rotation power (note: may need to be inverted).
            turn = -output;
        } else {
            // If the target is lost, reset the PID controller to prevent integral windup.
            headingController.reset();
            // no target found return done
            return true;
        }
        if (turn == 0) {
            return true;
        }

        follower.setTeleOpDrive(0, 0, turn, true);
        follower.update();

        return false;
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
            return true;
        } else if (id.id == AprilTagIds.TAG_ID_NONE.id) {
            // No tag is visible, update state to UNKNOWN.
            this.motifPattern = Motif.UNKNOWN;
        }

        // Return false if the pipeline is still switching, no tag is seen,
        // or a non-motif tag is visible.
        return false;
    }

    //==================================================================================================
    // State Control Methods
    //==================================================================================================

    /**
     * Starts the intake process. Stops the shooter, sets hood to intake position,
     * and runs the sweeper and spindexer.
     */
    public void startIntake() {
        currentState = SystemState.INTAKING;
        shooterSys.stop();
        kickerSys.setReady();
        hoodSys.setIntakePose();
        sweeperSys.startIntake();
        if (!isAutoIntaking) {
            spindexerSys.setFloat();
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
            hoodSys.setShootPose(); // Start moving hood to shoot position immediately.
        }
    }

    /**
     * Prepares the robot to shoot by spinning up the shooter wheels and setting the hood angle.
     */
    public boolean tryReadyShoot() {
        if (currentState == SystemState.IDLE) {
            currentState = SystemState.PREPPING_SHOOT;
            kickerSys.isReady();
            hoodSys.setShootPose();
            shooterSys.runShooter();
            return true;
        }

        return currentState == SystemState.PREPPING_SHOOT;
    }


    /**
     * Executes a shot if all subsystems are ready. Kicks a projectile and advances the spindexer.
     * This is a timed action that will transition the state to IDLE after completion.
     *
     * @return true if the shot was successfully initiated, false otherwise.
     */
    public boolean tryShoot() {
        if (isShootReady) {
            currentState = SystemState.SHOOTING;
            stateTimer.reset();
            kickerSys.kickIt();
            spindexerSys.setShootSlotEmpty();
            isShootReady = false; // Prevent immediate re-triggering.
            return true;
        }

        return false;
    }

    public boolean isSho
    public boolean continueShootMotif() {


        return spindexerSys.isEmpty();
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
                || !spindexerSys.isReady();
    }

    /**
     * Returns whether the robot is ready to shoot.
     *
     * @return The value of the volatile isShootReady flag.
     */
    public boolean getShootReady() {
        return this.isShootReady;
    }

    /**
     * Gets the path-following controller instance.
     *
     * @return The {@link Follower} object used for pathing and driving.
     */
    public Follower getFollower() {
        return follower;
    }

    //==================================================================================================
    // Passthrough and Utility Methods
    //==================================================================================================

    // TODO: Temp Code:
    public void resetHighLowColors() {
        colorSensorSys.resetHighLow();
    }

    public void setAutoIntaking(boolean autoIntaking) {
        isAutoIntaking = autoIntaking;
    }

    public void toggleAutoIntaking() {
        isAutoIntaking = !isAutoIntaking;
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
        return spindexerSys.isFull();
    }

    /** Sets the LED motif pattern. */
    public void setMotifPattern(Motif motifPattern) { this.motifPattern = motifPattern; }

    /** Manually advances the spindexer by one slot. */
    public void incrementSpindexerSlot() { spindexerSys.advanceOneSlot(); }

    /** Manually moves the spindexer back by one slot. */
    public void decrementSpindexerSlot() { spindexerSys.decreaseOneSlot(); }

    /** Rotates the spindexer to a specific slot index. */
    public void setSpindexerSlot(int index) { spindexerSys.rotateToSlot(index); }

    /** Increases the target shooter RPM for far shots. */
    public void increaseShooterRpmFar() { shooterSys.increaseRpmFar(); }

    /** Decreases the target shooter RPM for far shots. */
    public void decreaseShooterRpmFar() { shooterSys.decreaseRpmFar(); }

    /** Nudges the spindexer's raw position by a small positive amount. */
    public void increaseSpindexer() { spindexerSys.nudgePosition(5); }

    /** Nudges the spindexer's raw position by a small negative amount. */
    public void decreaseSpindexer() { spindexerSys.nudgePosition(-5); }

    /** Sets the shooter's target range to near or far. */
    public void setShooterTargetRange(boolean near) { shooterSys.setTargetRange(near); }

    /** Toggles the Limelight's active pipeline. */
    public void toggleLimelightTarget() { limelightSys.toggleTargetPileline(); }

    public void toggleFullLimelightPipeline(){ limelightSys.toggleFullPipeline(); }

    // ** Sets the Limelight's active pipeline. */
    public void setLimelightPipeline(Pipeline pipeline) { limelightSys.setPipeline(pipeline); }


    /**
     * Resets the IMU's heading. Catches and logs any interruption.
     */
    public void resetIMU() {
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
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
                        if (spindexerSys.getShootSlotState() != SlotState.EMPTY) {
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

            case SHOOTING:
                // After a short delay, reset the kicker to its ready position.
                if (stateTimer.milliseconds() >= SHOOT_KICKER_RESET_DELAY_MS) {
                    // Stop the shooter unless burst fire is active.
                    if (!isBurstFire) {
                        shooterSys.stop();
                    }

                    kickerSys.setReady();
                }

                // After a longer delay, advance the spindexer to the next slot and return to IDLE.
                if (stateTimer.milliseconds() >= SHOOT_SPINDEXER_ADVANCE_DELAY_MS) {
                    spindexerSys.advanceOneSlot();
                    currentState = SystemState.SPINDEXING;
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
                if (detectedState != SlotState.TRANSITIONING && hoodSys.isReadyToShoot()) {
                    SlotState currentSlotState = spindexerSys.getIntakeSlotState();
                    if ((detectedState != SlotState.UNKNOWN
                            && detectedState != currentSlotState)
                            || (detectedState == SlotState.UNKNOWN
                            && currentSlotState == SlotState.EMPTY)) {
                        spindexerSys.setIntakeSlotState(detectedState);
                    }
                }
            case INTAKING:
                if (isAutoIntaking) {
                    SlotState itemDetected = colorSensorSys.getLastDetection();

                    // Condition 1: An artifact is detected.
                    if (itemDetected != SlotState.EMPTY && hoodSys.isReadyInTake()) {

                        // First, check if we need to stop because the shooter slot is full.
                        if (spindexerSys.getShootSlotState() != SlotState.EMPTY) {
                            // This will close the hood and handle the state transition correctly.
                            this.stopIntake();
                            break; // Exit the INTAKING case
                        }

                        // if the spindexer is in float it can't be in the ready state
                        spindexerSys.setBrake();

                        // Condition 2: The spindexer is ready to move.
                        if (spindexerSys.isReady()) {

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
            case PREPPING_SHOOT:
                // No timed actions are needed in these states; they are controlled by user input.
                break;
        }
    }

    /**
     * Displays relevant robot data on the driver station via TelemetryManager.
     */
    private void displayTelemetry() {
        // TODO: remove unneeded output
        //telemetryM.addData("Shooter RPM", shooterSys.getCurrentRpm());
        //telemetryM.addData("Target RPM", shooterSys.getTargetRpm());
        telemetryM.addData("Robot State", currentState.name());
        telemetryM.addData("Shoot Slot Index", spindexerSys.getCurShootSlot());
        telemetryM.addData("Shoot Slot State:", spindexerSys.getShootSlotState());
        telemetryM.addData("Intake Slot State:", spindexerSys.getIntakeSlotState());
        telemetryM.addData("Is Auto Intaking:", isAutoIntaking);
        telemetryM.addData("Hood State:", hoodSys.getHoodState());

        //telemetryM.addData("Shooter Ready:", shooterSys.isReadyToShoot());
        //telemetryM.addData("Spindexer Raw Position", spindexerSys.getCurrentPosition());
        //telemetryM.addData("Spindexer Slot", spindexerSys.getCurrentSlot());
        //telemetryM.addData("Shoot Ready:", isShootReady);
        telemetryM.update(telemetry);
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
        // Set the PID controller's target to 0 (no error).
        headingController.setTargetPosition(0);
        // Update the controller with the current error.
        headingController.updateError(txDelta);
        // Run the PID calculation.
        double pidOutput = headingController.run();

        // If the error is outside the tolerance, return the clamped PID output.
        // Otherwise, return 0 to stop turning.
        return Math.abs(headingController.getError()) > tolerance
                ? MathFunctions.clamp(pidOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER)
                : 0;
    }
}