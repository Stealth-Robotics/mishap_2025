package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.PathMid6Ball;
import org.firstinspires.ftc.teamcode.paths.PathMid6BallPark;
import org.firstinspires.ftc.teamcode.paths.util.Path;
import org.firstinspires.ftc.teamcode.paths.PathMidShot;
import org.firstinspires.ftc.teamcode.paths.util.PathState;

import java.util.Arrays;

public class AutoMid6Ball extends AutosDecode {

    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 4));
        intakeIndexes.addAll(Arrays.asList(3));
        return new PathMid6BallPark(robot);
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
            this.aimOffset = -1;
        } else {
            this.aimOffset = 1;
        }

        this.aimTolerance = 0.6;

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
