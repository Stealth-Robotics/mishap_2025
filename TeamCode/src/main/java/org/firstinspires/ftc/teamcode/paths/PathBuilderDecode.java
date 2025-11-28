package org.firstinspires.ftc.teamcode.paths;

import static org.firstinspires.ftc.teamcode.paths.PathManager.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Curve;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

/**
 * Extends PathBuilder to add some custom functionality for Decode.
 * Note: that because PathBuilder doesn't expose paths we had to get
 * inventive some of the calls (such as addCallback)
 */
public class PathBuilderDecode extends PathBuilder {

    private final RobotSystem robot;

    public PathBuilderDecode(RobotSystem robot) {
        super(robot.getFollower());
        this.robot = robot;
    }

    /**
     * Checks the intake slot for an artifact and the robot is intaking
     * @return True if intaking should be paused, False otherwise
     */
    protected PathBuilder.CallbackCondition shouldPauseIntaking() {
        return () -> robot.isIntaking() && robot.getColorSensorDetectedArtifact() && robot.isEggbeaterRunning();
    }

    /**
     * Checks to see if intaking should resume when intaking artifacts
     * @return True if intaking should resume, False otherwise
     */
    protected PathBuilder.CallbackCondition shouldResumeIntaking() {
        return ()-> robot.isIntaking() && !robot.getColorSensorDetectedArtifact() && !robot.isEggbeaterRunning();
    }

    /**
     * pauses the intake and sets the speed to slow to allow for the spindexer to rotate
     * @return A runnable that pauses the intake and sets the speed to slow
     */
    protected Runnable pauseIntaking() {
        return () -> {
            robot.stopEggbeater();
            robot.getFollower().setMaxPower(SLOW_SPEED);
        };
    };

    /**
     * resumes the intake and sets the speed to the max speed
     * @return A runnable that resumes the intake and sets the speed to the max speed
     */
    protected Runnable resumeIntaking() {
        return () -> {
            robot.startEggbeater();
            robot.getFollower().setMaxPower(INTAKE_SPEED);
        };
    }

    /**
     * Applies a standard intake sequence to a path builder.
     * This sequence starts the intake, sets a slower speed, and adds conditional callbacks
     * for pausing/resuming based on artifact detection and stopping when the spindexer is full.
     *
     * @return The modified PathBuilder with the intake sequence added.
     */
    public PathBuilderDecode applyIntakeSequence() {
        Follower follower = robot.getFollower();
        addParametricCallback(.01, robot::startIntake)
        .addParametricCallback(.1, ()->follower.setMaxPower(INTAKE_SPEED))
        .addParametricCallback(.99, robot::stopIntake)
        .addCallback(shouldPauseIntaking(), pauseIntaking())
        .addCallback(shouldResumeIntaking(), resumeIntaking())
        .addCallback(robot::isSpindexerFull, () -> {
            robot.stopIntake();
            follower.setMaxPower(MAX_SPEED);
            // this causes the follower to pickup the next path from the closest point
            follower.breakFollowing();
        });

        return this;
    }
    /**
     * Applies a standard shooting sequence to a path builder.
     * This sequence ensures the robot is at max speed, sorts the spindexer if needed,
     * selects the first target, and starts the shooter wheels.
     *
     * @return The modified PathBuilder with the shooting sequence added.
     */
    public PathBuilderDecode applyFirstShotSequence(ZoneDistance distance) {
        addParametricCallback(0.01, robot::trySelectFirstMotifSlot)
        .addParametricCallback(.99, ()-> {
            robot.setShooterTargetRange(distance);
            robot.startShooter();
        });

        return this;
    }

    /**
     * Applies the followup shot sequence to the path. Primary differince is
     * more sorting is done for uknown states etc.
     * @param distance The distance to shoot
     * @return The modified PathBuilder with the followup shot sequence added.
     */
    public PathBuilderDecode applyFollowupShotSequence(ZoneDistance distance) {
        Follower follower = robot.getFollower();
        addParametricCallback(0.01, () -> {
            follower.setMaxPower(MAX_SPEED);
            robot.stopIntake(); // just incase
        })
        // Check in on the spindexer to see if there are any unknown artifacts and try to sort them
        .addCallback(
                () -> !robot.isSpindexerBusy()
                        && robot.isAnyArtifactUnknown()
                        && robot.isHoodShootPose(),
                robot::incrementSpindexerSlot, 3)

        .addParametricCallback(.7, robot::trySelectFirstMotifSlot)
        .addParametricCallback(.99, ()-> {
            robot.setShooterTargetRange(distance);
            robot.startShooter();
        });
        return this;
    }

    //*****************************************************************************************
    //
    // Passthrough overrides to return the correct object type
    //
    //****************************************************************************************

    @Override
    public PathBuilderDecode addPath(Path path) {
        super.addPath(path);
        return this;
    }

    @Override
    public PathBuilderDecode addPath(Curve curve) {
        super.addPath(curve);
        return this;
    }

    @Override
    public PathBuilderDecode setLinearHeadingInterpolation(double startHeading, double endHeading) {
        super.setLinearHeadingInterpolation(startHeading, endHeading);
        return this;
    }

   @Override
    public PathBuilderDecode setConstantHeadingInterpolation(double setHeading) {
        super.setConstantHeadingInterpolation(setHeading);
        return this;
    }

    @Override
    public PathBuilderDecode setTangentHeadingInterpolation() {
        super.setTangentHeadingInterpolation();
        return this;
    }

    @Override
    public PathBuilderDecode setReversed() {
        super.setReversed();
        return this;
    }

    @Override
    public PathBuilderDecode addParametricCallback(double t, Runnable runnable) {
        super.addParametricCallback(t, runnable);
        return this;
    }

    @Override
    public PathBuilderDecode addCallback(PathBuilder.CallbackCondition condition, Runnable runnable, int count) {
        super.addCallback(condition, runnable, count);
        return this;
    }

    /**
     * Changes the default behavoir of addCallback to make the Finite action run closer to Infinite
     * @param condition The condition that must be met for the callback to run.
     * @param runnable The action to run when the condition is met.
     * @return The modified PathBuilderDecode with the new callback added.
     */
    @Override
    public PathBuilderDecode addCallback(PathBuilder.CallbackCondition condition, Runnable runnable) {
        super.addCallback(condition, runnable, Integer.MAX_VALUE);
        return this;
    }

}