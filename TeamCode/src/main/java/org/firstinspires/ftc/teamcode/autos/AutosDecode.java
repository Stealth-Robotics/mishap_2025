package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.FinalPose;
import org.firstinspires.ftc.teamcode.common.Motif;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathManager;
import org.firstinspires.ftc.teamcode.paths.PathState;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

import java.util.HashSet;

/**
 * base auto class for decode 25/26
 */
public abstract class AutosDecode extends OpMode {

    public static final double READ_MOTIF_TIMEOUT_SECONDS = 5;
    public static final double AIM_TIMEOUT_MS = 4000;
    public static final double AIM_MIN_MS = 2000;

    public static final double INTAKE_WAIT_MS = 1000;

    protected static final double PIPELINE_SWITCH_DELAY = 1000;
    protected final ElapsedTime actionTimer = new ElapsedTime();
    protected static final long INTAKE_DELAY = 5000; // delay to keep hood open
    protected final HashSet<Integer> shootIndexes = new HashSet<>();
    protected final HashSet<Integer> intakeIndexes = new HashSet<>();
    protected int subActionStep = 0;
    protected int lastPathIndex = -1;

    protected double startWaitTimeSeconds = 0;

    protected TelemetryManager telemetryM;
    protected Follower follower;
    protected RobotSystem robot;
    protected PathState pathState = PathState.READ_MOTIF;
    protected Path paths;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean isSpindexerReady = false;

    private boolean areArtifactsSorted = false;

    protected double aimOffset = 0;

    protected boolean waitTriggered = false;


    // --- Abstract Methods to be Implemented by Child Classes ---

    /**
     * Initializes the specific paths for this autonomous routine.
     * @return The configured Path object.
     */
    protected abstract Path initPaths();

    /** Last pose that was estimated by the limelight*/
    protected Pose lastPose;


    /**
     * Called once when the op mode is initialized.
     */
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new RobotSystem(hardwareMap, telemetry);
        follower = robot.getFollower();
        setSpindexerInitState();
        telemetryM.debug("Status", "Initialized");
        robot.setLimelightPipeline(Pipeline.APRILTAG_TARGET_BOTH);
        robot.update();
    }

    /**
     * override if you want to manually setup the spindexer slot
     * to prevent the rotation of the spindexer during init
     */
    protected void setSpindexerInitState()
    {
        //robot.initSpindxerSlotsEmpty();
        robot.initSpindxerSlotsAuto();
    }

    /**
     * Called repeatively after init but before start.
     */
    @Override
    public void init_loop() {
        robot.update();
        if (!isSpindexerReady) {
            isSpindexerReady = robot.doInitSpindexer();
        } else if (!areArtifactsSorted) {
            areArtifactsSorted = robot.doArtifactSort();
        }


        // TODO: Use this to get robot position from limelight while waiting
        telemetryM.addData("Limelight Pipeline:", robot.getLimelightPipeline());
        Pose curPose = robot.getPedroPoseFromLimelight(800);
        if (curPose != null) {
            this.lastPose = curPose;
            telemetryM.addData("Alliance: ", PathManager.getAllianceFromPose(curPose) == Alliance.RED ? "RED" : "BLUE");
            telemetryM.addData("Last Pose X", curPose.getX());
            telemetryM.addData("Last Pose Y", curPose.getY());
            telemetryM.addData("Last Pose Heading:", Math.toDegrees(curPose.getHeading()));

        }
    }

    /**
     * Sets the alliance for this autonomous routine.
     * Use limitlight if not overriden.
     */
    protected void setAlliance() {
        // this means if we don't get a pose we will default to RED alliance
        if (lastPose != null) {
            PathManager.setAlianceFromPose(lastPose);
        }
    }

    protected abstract void setSpindexerSlots();

    /**
     * Allows for overriding the start Pose of the follower.
     * By default we will attempt to get the limelight Pose and use that
     */
    protected void setStartingPose() {
        if (lastPose != null) {
            follower.setStartingPose(lastPose);
        }
        else {
            follower.setStartingPose(paths.getPathStart());
        }
    }


    /**
     * Called once when the start button is pressed.
     */
    @Override
    public void start() {
        robot.update();
        // TODO: Any addtional 1 time actions when start button is pressed

        // must be called before initPaths
        setAlliance();
        paths = initPaths();
        // must be called after initPaths.
        setStartingPose();

        //telemetryM.addData("Paths Count", paths.getSegmentCount());

        // this means the hood will stay open during intaking
        robot.setAutoIntaking(true);
        stateTimer.reset();
    }

    /**
     * all state machine logic is here
     */
    @Override
    public void loop() {
        // robot.update calls follower.update
        robot.update();

        // Protect the robot from early start
        if (!isSpindexerReady) {
            telemetryM.addLine("Waiting for Spindexer JJ!!!");
            isSpindexerReady = robot.doInitSpindexer();
            return;
        }

        if (!robot.isSpindexerFull() && !this.areArtifactsSorted) {
            areArtifactsSorted = robot.doArtifactSort();
            telemetryM.addLine("Waiting for Artifacts!!!");
            return;
        }

        if (stateTimer.seconds() > startWaitTimeSeconds
            && !waitTriggered) {
            waitTriggered = true;
            stateTimer.reset();
            return;
        }

        telemetryM.addData("Current State", pathState);
        switch (pathState) {
            case READ_MOTIF:
                // wait till the last minute to read the obelisk to get the motif
                if (!this.doMotifOrTimeout()) {
                    return;
                }

                pathState = PathState.SET_TARGET;
                break;
            case SET_TARGET:
                // set the target for correct aliance
                robot.setLimelightPipeline(Alliance.isRed()
                        ? Pipeline.APRILTAG_TARGET_RED
                        : Pipeline.APRILTAG_TARGET_BLUE);
                pathState = PathState.IDLE;
                break;
            case IDLE:
                pathState = checkIndexForAction();
                if (pathState == PathState.IDLE || pathState == PathState.CONTINUE) {
                    pathState = updateFollowerState();
                }

                break;
            case BUSY:
                if (!follower.isBusy()) {
                    pathState = PathState.IDLE;
                }

                break;
            case WAIT:
                pathState = checkIndexForAction();
                break;
            case STOP:
                follower.breakFollowing();
                FinalPose.setPose(follower.getPose());
                requestOpModeStop();
                return;
            case CONTINUE:
                pathState = updateFollowerState();
                break;
        }

        telemetryM.addData("Path State", pathState);
        //telemetryM.addData("Follower State:", follower.isBusy());
        telemetryM.addData("Path Index", paths.getSegmentIndex());
    }

    /**
     * Checks if an action needs to be performed at the current path index.
     * This method can be overridden by subclasses to add more complex actions.
     */
    protected PathState checkIndexForAction() {
        int curIndex = paths.getSegmentIndex();
        if (curIndex != lastPathIndex) {
            subActionStep =0;
            lastPathIndex = curIndex;
        }

        if (shootIndexes.contains(curIndex)) {
            telemetryM.addData("SubAction State", subActionStep);
            switch (subActionStep) {
                case 0:
                    // set the subindex to the correct next step
                    subActionStep = startAiming() ? 1 : 2;
                    return PathState.WAIT;
                case 1:
                    if (doAimingOrTimeout()) {
                        ++subActionStep;
                    }
                    return PathState.WAIT;
                case 2:
                    // Ready for Motif shot if false there is nothing to shoot
                        if (robot.startShootMotif(true)) {
                            subActionStep++;
                            return PathState.WAIT;
                        }

                    break;
                case 3:
                    // Try to shoot the artifact
                    if (!robot.continueShootMotif()) {
                        return PathState.WAIT;
                    }

                    break;
                default:
                    subActionStep = -1;
                    return PathState.CONTINUE;
            }
        } else if (intakeIndexes.contains(curIndex)) {
           switch (subActionStep) {
               case 0:
                   actionTimer.reset();
                   ++subActionStep;
                   return PathState.WAIT;
               case 1:
                    return doIntakeAction() ? PathState.CONTINUE : PathState.WAIT;
           }
        }

        // Default implementation can be empty or handle common actions.
        // Specific actions are now in the overridden method in the child class.
        return PathState.IDLE;
    }

    /**
     * Any acctions required durring the intake phase
     * @return true once done otherwise false
     */
    protected boolean doIntakeAction()
    {
        if (stateTimer.milliseconds() < INTAKE_WAIT_MS) {
            return false;
        }

        if (!robot.isSpindexerBusy() || stateTimer.milliseconds() > INTAKE_DELAY) {
            robot.stopIntake();
            return true;
        }
        // hold up if the spindexer is busy but not too long
        return false;
    }

    /**
     * Attempt to get the current motif from the obelisk
     * @return false for still looking true for found or timed out.
     */
    protected boolean doMotifOrTimeout() {
        // If we timout set the mofif to the loaded pattern of GPP
        if (stateTimer.seconds() > READ_MOTIF_TIMEOUT_SECONDS) {
            robot.setMotifPattern(Motif.PPG);
            return true;
        }
        boolean ret = robot.doReadMotif();
        if (ret) {
            // update the static motif variable
            Motif.set(robot.getDetectedMotif());
        }

        return ret;
    }

    /**
     * Starts aiming at the target.
     * adjust latency based on the path segment setTimeoutConstraint
     * @return true if done aiming otherwise false
     */
    public boolean startAiming() {
        if (robot.isSpindexerEmpty()) {
            return false;
        }

        stateTimer.reset();

        robot.doAimAtTarget(.1,  aimOffset,50);
        return true;
    }

    /**
     * Continues the aiming operation until the timeout is reached.
     * @return true for done aiming otherwise false
     */
    protected boolean doAimingOrTimeout() {
        boolean done = false;
        double curTimeMs = stateTimer.milliseconds();
        if (curTimeMs > AIM_TIMEOUT_MS) {
            // stop the robot if timed out.
            follower.setTeleOpDrive(0, 0, 0, true);
            done = true;
        }
       else {
            done = robot.doAimAtTarget(.2, aimOffset, 100);
        }

        return done && curTimeMs > AIM_MIN_MS;
    }

    /**
     * Keeps the follower moving and tracking the path.
     *
     * @return The current state of the follower.
     */
    private PathState updateFollowerState() {
        if (pathState == PathState.WAIT) {
            return pathState;
        }

        if (follower.isBusy()) {
            return PathState.BUSY;
        }

        PathChain curPath = paths.getNextSegment();
        if (curPath == null) {
            return PathState.STOP;
        }


        follower.followPath(curPath, true);
        return follower.isBusy() ? PathState.BUSY : PathState.IDLE;
    }
}
