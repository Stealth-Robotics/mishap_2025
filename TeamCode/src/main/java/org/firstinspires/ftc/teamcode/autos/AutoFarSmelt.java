package org.firstinspires.ftc.teamcode.autos;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathFarAuto1;
import org.firstinspires.ftc.teamcode.paths.PathFarSmelt;
import org.firstinspires.ftc.teamcode.paths.PathState;

import java.util.Arrays;

//@Autonomous(name = "Shoot Far either Side", group = "Autonomous", preselectTeleOp = "_TeleOp_Driver_Operator")
//@Configurable
//@Disabled
public class AutoFarSmelt extends AutosDecode {


    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 4));
        intakeIndexes.addAll(Arrays.asList(3));

        return new PathFarSmelt(robot);
    }

    @Override
    protected void setSpindexerSlots(){

        robot.initSpindxerSlotsEmpty();
    }

    /**
     * This is overriding using the limelight as the start Pose
     * this can be removed once limelight is tuned
     */
    @Override
    protected void setStartingPose() {

        // cahnge the angle of the far shots by a couple of degrees:
        // a negative number turns the bot more to the left positive more to the right
        if (Alliance.isBlue()) {
            this.aimOffset = -3.5;
        } else {
            this.aimOffset = 3.5;
        }

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
