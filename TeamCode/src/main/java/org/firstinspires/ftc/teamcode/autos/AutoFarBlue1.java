package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;

import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathFarAuto1;
import org.firstinspires.ftc.teamcode.paths.PathState;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

// TODO: move a lot of this into a base class and extend it
@Autonomous(name = "Far Blue1", group = "Autonomous", preselectTeleOp = "_TeleOp Driver Only")
@Configurable // Panels
public class AutoFarBlue1 extends OpMode {

    private TelemetryManager telemetryM; // Panels Telemetry instance
    private Follower follower; // Pedro Pathing follower instance
    private RobotSystem robot;
    private PathState pathState = PathState.IDLE;
    private Path paths;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        // this should be getting updated based on the path POSE
        Alliance.set(Alliance.BLUE);

        robot = new RobotSystem(hardwareMap, telemetry);

        follower = robot.getFollower();

        paths = new PathFarAuto1(robot); // Build paths

        // Sets the rough starting spot can update with limelight in the wait to start loop
        follower.setStartingPose(paths.getStartPose());

        telemetryM.debug("Status", "Initialized");
        telemetryM.addData("Paths Count", paths.getSegmentCount());

        robot.update();
    }

    @Override
    public void init_loop() {
        robot.update();
        //TODO: Use this time to check for obelisk and update paths
    }

    @Override
    public void start() {
        robot.update();
        // TODO: Rotate the Spindexer to correct slot
    }

    @Override
    public void loop() {
        // Update robot systems and the path follower
        robot.update();

        switch (pathState) {
            case IDLE:
                // If idle, check if we need to perform an action (like shooting)
                // or start the next path segment.
                pathState = checkIndexForAction();

                // If checkIndexForAction() didn't change the state to WAIT,
                // then it's time to move to the next path segment.
                if (pathState == PathState.IDLE || pathState == PathState.CONTINUE) {
                    pathState = updateFollowerState();
                }
                break;

            case BUSY:
                // If the follower is busy, we just need to wait for it to finish.
                // robot.update() (called above) handles the actual follower updates.
                if (!follower.isBusy()) {
                    // The path segment is complete, go back to IDLE to decide what's next.
                    pathState = PathState.IDLE;
                }
                break;

            case WAIT:
                // The robot is waiting for a subsystem (like the shooter).
                // Check if the action is now complete.
                pathState = checkIndexForAction(); // This method should return CONTINUE when ready.
                break;

            case STOP:
                // All paths are done, stop the OpMode.
                requestOpModeStop();
                return;

            case CONTINUE:
                // CONTINUE is used when completing a specific index task
                pathState = updateFollowerState();
                break;
        }

        // Log values to Panels and Driver Station
        telemetryM.addData("Path State", pathState);
        telemetryM.addData("Follower State:", follower.isBusy());
        telemetryM.addData("Path Index", paths.getSegmentIndex());
    }

    private PathState checkIndexForAction() {

        if (robot.isIntakeRunning())
        {
            // TODO: check color sensor etc.
            robot.stopIntake();
            return PathState.WAIT;
        }

        int curIndex = paths.getSegmentIndex();
        if (curIndex == 1 || curIndex == 7) {
            if (robot.getShootReady()) {
                if (robot.shoot()) {
                    return PathState.CONTINUE;
                } else {
                    telemetryM.addLine("Not READY");
                }
            }
            else {
                robot.setReadyShoot();
                return PathState.WAIT;
            }
        }

        return PathState.IDLE;
    }

    private PathState updateFollowerState() {

        if (pathState == PathState.WAIT) {
            return pathState;
        }

        if (follower.isBusy())
        {
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