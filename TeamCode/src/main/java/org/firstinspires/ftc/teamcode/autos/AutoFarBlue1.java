package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathFarAuto1;
import org.firstinspires.ftc.teamcode.paths.PathState;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

import java.util.Arrays;
import java.util.HashSet;

@Autonomous(name = "Far Blue 1", group = "Autonomous", preselectTeleOp = "_TeleOp Driver Only")
@Configurable
public class AutoFarBlue1 extends AutosDecode {
    // Timer for handling delays
    private final ElapsedTime actionTimer = new ElapsedTime();
    private static final long SHOOT_SEQUENCE_DELAY_MS = 1500; // 0.5 second delay

    private int subActionStep = 0;
    private int lastPathIndex = -1;

    // Define HashSets for your action indices
    private final HashSet<Integer> shootIndexes = new HashSet<>(Arrays.asList(1, 7));
    private final HashSet<Integer> intakeIndexes = new HashSet<>(Arrays.asList(3, 4, 5));


    @Override
    protected Path initPaths(RobotSystem robot) {
        // This is now the only place where you define the specific path class
        return new PathFarAuto1(robot);
    }

    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        Alliance.set(Alliance.BLUE);
    }

    @Override
    protected PathState checkIndexForAction() {
        // First, call the base implementation for any common actions
        PathState baseState = super.checkIndexForAction();
        if (baseState != PathState.IDLE) {
            return baseState; // The base class is handling something
        }

        // Now, add actions specific to AutoFarBlue1
        int curIndex = paths.getSegmentIndex();
        if (curIndex != lastPathIndex) {
            subActionStep =0;
            lastPathIndex = curIndex;
        }

        if (shootIndexes.contains(curIndex)) {

            switch (subActionStep) {
                case 0:
                    subActionStep = startAiming() ? 2 : 1;
                    return PathState.WAIT;
                case 1:
                    if (doAiming()) {
                        ++subActionStep;
                    }
                    return PathState.WAIT;
                case 2:
                    if (robot.getShootReady()) {
                        if (robot.shootArtifact()) {
                            return PathState.CONTINUE;
                        }
                    } else {
                        robot.setReadyShoot();
                        return PathState.WAIT;
                    }

                    break;
                default:
                    subActionStep = -1;
                    return PathState.CONTINUE;
            }
        } else if (intakeIndexes.contains(curIndex)) {
            // TODO: Add intake actions here
        }

        return PathState.IDLE; // No action taken
    }
}
