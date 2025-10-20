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
@Autonomous(name = "Far Blue1", group = "Autonomous")
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

        // TODO can update via Limelight
        follower.setStartingPose(new Pose(56, 8.5, Math.toRadians(90)));

        paths = new PathFarAuto1(robot); // Build paths

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
        // Update robot systems
        robot.update();

        if (pathState == PathState.STOP) {
            requestOpModeStop();
            return;
        }

        if (pathState == PathState.IDLE || pathState == PathState.WAIT) {
            pathState = checkIndexForAction();
        }

        if (pathState == PathState.IDLE) {
            pathState = updateFollowerState();
        }

        // Log values to Panels and Driver Station
        telemetryM.debug("Path State", pathState);
        telemetryM.update(telemetry);
    }

    private PathState checkIndexForAction() {
        int curIndex = paths.getSegmentIndex();
        if (curIndex == 1 || curIndex == 6) {
            if (robot.getShootReady()) {
                if (robot.shoot()) {
                    return PathState.IDLE;
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