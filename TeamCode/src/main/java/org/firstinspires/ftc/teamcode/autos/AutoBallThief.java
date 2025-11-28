package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathBallThief;

import java.util.Arrays;

public class AutoBallThief  extends AutosDecode{

    /**
     * Initializes the specific paths for this autonomous routine.
     *
     * @return The configured Path object.
     */
    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 4, 7));
        intakeIndexes.addAll(Arrays.asList(3,6));
        return new PathBallThief(robot);
    }

    /**
     *
     */
    @Override
    protected void setSpindexerSlots() {
        robot.initSpindxerSlotsAuto();
    }

    @Override
    protected void setStartingPose() {

        // cahnge the angle of the far shots by a couple of degrees:
        // a negative number turns the bot more to the left positive more to the right
        if (Alliance.isBlue()) {
            this.aimOffset = -2.5;
        } else {
            this.aimOffset = 2.5;
        }

        follower.setStartingPose(paths.getPathStart());
    }
}
