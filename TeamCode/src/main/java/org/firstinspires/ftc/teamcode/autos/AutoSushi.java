package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathNearAuto1;
import org.firstinspires.ftc.teamcode.paths.PathState;

import java.util.Arrays;

public class AutoSushi extends AutosDecode {

    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1));
        //intakeIndexes.addAll(Arrays.asList(2));
        return new PathNearAuto1(robot);
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
            this.aimOffset = 0;
        } else {
            this.aimOffset = 0;
        }

        Pose startPose = paths.getPathStart();
        if (lastPose != null) {
            startPose = startPose.setHeading(lastPose.getHeading());
        }
        follower.setStartingPose(startPose);
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
