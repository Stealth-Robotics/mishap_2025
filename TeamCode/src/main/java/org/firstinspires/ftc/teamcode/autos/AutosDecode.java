package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathState;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

/**
 * base auto class for decode 25/26
 */
public abstract class AutosDecode extends OpMode {

    public static final double MOTIF_TIMEOUT = 2000;
    public static final double AIM_TIMEOUT = 1000;
    private final ElapsedTime actionTimer = new ElapsedTime();
    private static final long INTAKE_DELAY = 1500; // 0.5 second delay

    protected TelemetryManager telemetryM;
    protected Follower follower;
    protected RobotSystem robot;
    protected PathState pathState = PathState.READ_MOTIF;
    protected Path paths;
    private final ElapsedTime stateTimer = new ElapsedTime();

    private boolean isSpindexerReady = false;


    // --- Abstract Methods to be Implemented by Child Classes ---

    /**
     * Initializes the specific paths for this autonomous routine.
     * @param robot The robot system instance.
     * @return The configured Path object.
     */
    protected abstract Path initPaths(RobotSystem robot);

    /**
     * Sets the alliance for this autonomous routine.
     */
    protected abstract void setAlliance();

    // --- Core Logic Moved from AutoFarBlue1 ---

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        robot = new RobotSystem(hardwareMap, telemetry);
        follower = robot.getFollower();

        // Call abstract methods to get specific configurations
        setAlliance();
        paths = initPaths(robot);

        follower.setStartingPose(paths.getStartPose());

        telemetryM.debug("Status", "Initialized");
        telemetryM.addData("Paths Count", paths.getSegmentCount());

        robot.update();
    }

    @Override
    public void init_loop() {
        robot.update();
        if (!isSpindexerReady) {
            isSpindexerReady = robot.initSpindexer();
        }

        // TODO: Use this to get robot position from limelight while waiting
    }

    @Override
    public void start() {
        robot.update();

        // TODO: Can be overridden for start-specific actions
        stateTimer.reset();
    }

    /**
     * all state machine logic is here
     */
    @Override
    public void loop() {
        robot.update();

        // Protect the robot from early start
        if (!isSpindexerReady) {
            isSpindexerReady = robot.initSpindexer();
            return;
        }

        switch (pathState) {
            case READ_MOTIF:
                if (!this.findMotifOrTimeout()) {
                    return;
                }

                pathState = PathState.SET_TARGET;
                break;
            case SET_TARGET:
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
                requestOpModeStop();
                return;
            case CONTINUE:
                pathState = updateFollowerState();
                break;
        }

        telemetryM.addData("Path State", pathState);
        telemetryM.addData("Follower State:", follower.isBusy());
        telemetryM.addData("Path Index", paths.getSegmentIndex());
    }

    /**
     * Checks if an action needs to be performed at the current path index.
     * This method can be overridden by subclasses to add more complex actions.
     */
    protected PathState checkIndexForAction() {
        if (robot.isIntakeRunning()) {
            // TODO: check color sensor etc.
            robot.stopIntake();
            return PathState.WAIT;
        }

        // Default implementation can be empty or handle common actions.
        // Specific actions are now in the overridden method in the child class.
        return PathState.IDLE;
    }

    protected boolean findMotifOrTimeout() {
        if (stateTimer.milliseconds() > MOTIF_TIMEOUT) {
            return true;
        }

        return robot.readMotif();
    }

    /**
     * Starts aiming at the target.
     * adjust latency based on the path segment setTimeoutConstraint
     * @return true if done aiming otherwise false
     */
    public boolean startAiming() {
        stateTimer.reset();
        return robot.tryAimAtTarget(1, 500);
    }

    /**
     * Continues the aiming operation until the timeout is reached.
     * @return true for done aiming otherwise false
     */
    protected boolean doAiming() {
        if (stateTimer.milliseconds() > AIM_TIMEOUT) {
            return true;
        }

        return robot.tryAimAtTarget(1, 500);
    }

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

        follower.followPath(curPath);
        return follower.isBusy() ? PathState.BUSY : PathState.IDLE;
    }
}
