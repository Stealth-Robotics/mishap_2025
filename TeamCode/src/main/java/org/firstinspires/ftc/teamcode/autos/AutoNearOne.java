package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathNearAuto1;
import org.firstinspires.ftc.teamcode.paths.PathState;

import java.util.Arrays;
@Autonomous(name = "Shoot Near either Side", group = "Autonomous", preselectTeleOp = "_TeleOp_Driver_Only")
@Configurable
public class AutoNearOne extends AutosDecode {

    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 4));
        intakeIndexes.addAll(Arrays.asList(3));
        return new PathNearAuto1(robot);
    }

    @Override
    protected void setSpindexerSlots() {
        robot.initSpindxerSlotsEmpty();
    }

//    @Override
//    protected void setStartingPose() {
//        Pose startPose = paths.getPathStart();
//        follower.setStartingPose(paths.getPathStart());
//    }

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
