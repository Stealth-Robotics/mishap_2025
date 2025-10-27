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

@Autonomous(name = "Shoot Far either Side", group = "Autonomous", preselectTeleOp = "_TeleOp Driver Only")
@Configurable
public class AutoFarBlue1 extends AutosDecode {

    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 7));
        intakeIndexes.addAll(Arrays.asList(6));

        return new PathFarAuto1(robot);
    }

    // TODO: uncomment if you don't want limelight to be used as aliance positioning
//    @Override
//    protected void setAliance() {
//        // Set the specific alliance for this OpMode
//        // can use limelight data if you want
//        Alliance.set(Alliance.BLUE);
//    }

    /**
     * This is overriding using the limelight as the start Pose
     * this can be removed once limelight is tuned
     */
    @Override
    protected void setStartingPose() {
        follower.setStartingPose(paths.getPathStart());
    }

    @Override
    protected PathState checkIndexForAction() {
        // First, call the base implementation for any common actions
        PathState baseState = super.checkIndexForAction();
        if (baseState != PathState.IDLE) {
            return baseState; // The base class is handling something
        }

        // TODO: Add any specific actions here for the auto here
        // shooting and intaking are in in the base class

        return PathState.IDLE;
    }
}
