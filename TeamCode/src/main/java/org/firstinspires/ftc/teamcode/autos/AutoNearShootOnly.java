package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.PathNearShootOnly;
import org.firstinspires.ftc.teamcode.paths.util.Path;
import org.firstinspires.ftc.teamcode.paths.PathNear6Ball;
import org.firstinspires.ftc.teamcode.paths.util.PathState;

import java.util.Arrays;

public class AutoNearShootOnly extends AutosDecode {

    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1));
        //intakeIndexes.addAll(Arrays.asList(2));
        return new PathNearShootOnly(robot);
    }

    @Override
    protected void setSpindexerSlots() {
        robot.initSpindxerSlotsEmpty();
    }

    @Override
    protected void setStartingPose() {
        // change the angle of the far shots by a couple of degrees:
        // a negative number turns the bot more to the left positive more to the right
        if (Alliance.isBlue()) {
            this.aimOffset = 1;
        } else {
            this.aimOffset = -1;
        }

        Pose startPose = paths.getPathStart();
        if (lastPose != null) {
            startPose = startPose.setHeading(lastPose.getHeading());
        }
        follower.setStartingPose(startPose);
        startWaitTimeSeconds = 5;
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
