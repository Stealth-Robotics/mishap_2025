package org.firstinspires.ftc.teamcode.paths;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages a sequence of path segments for autonomous robot navigation.
 * This class holds separate path sequences for the Red and Blue alliances
 * and provides methods to step through them. It also sets the robot's
 * starting pose and determines the alliance based on that pose.
 */
public class PathManager implements Path {

    /** The starting pose of the robot on the field. */
    protected Pose startPose;

    /** A list of path segments for the Red alliance. */
    protected final List<PathChain> redPathSegments = new ArrayList<>();

    /** A list of path segments for the Blue alliance. */
    protected final List<PathChain> bluePathSegments = new ArrayList<>();

    /** The index of the current path segment being executed. */
    protected int currentPath = 0;

    /** The robot system this path manager controls. */
    protected final RobotSystem robot;

    /**
     * Constructs a new PathManager.
     *
     * @param robot The instance of the {@link RobotSystem} this manager will interact with.
     */
    public PathManager(RobotSystem robot) {
        this.robot = robot;
    }

    /**
     * Retrieves the next path segment in the sequence for the current alliance.
     * Increments the path index after retrieval.
     *
     * @return The next {@link PathChain} segment, or {@code null} if all segments have been completed.
     */
    @Nullable
    public PathChain getNextSegment() {
        if (Alliance.isRed()) {
            if (currentPath >= redPathSegments.size()) {
                return null;
            }
            return redPathSegments.get(currentPath++);
        } else {
            if (currentPath >= bluePathSegments.size()) {
                return null;
            }
            return bluePathSegments.get(currentPath++);
        }
    }

    /**
     * Gets the total number of path segments for the current alliance.
     *
     * @return The number of path segments.
     */
    public int getSegmentCount() {
        return Alliance.isRed() ? redPathSegments.size() : bluePathSegments.size();
    }

    /**
     * Gets the index of the current path segment.
     *
     * @return The zero-based index of the current segment.
     */
    @Override
    public int getSegmentIndex() {
        return currentPath;
    }

    /**
     * Sets the starting pose for the robot and configures the alliance.
     * This method updates the path follower with the start pose and determines
     * the alliance based on the robot's starting X-coordinate on the field.
     *
     * @param startPose The {@link Pose} to set as the robot's starting position.
     */
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
        robot.getFollower().setStartingPose(this.startPose);

        // Determine alliance based on which side of the field the robot starts on.
        // Assumes Red is on the positive X side.
        if (startPose.getX() > (Constants.FIELD_SIZE_X_INCHES / 2.0)) {
            Alliance.set(Alliance.RED);
        } else {
            Alliance.set(Alliance.BLUE);
        }
    }

    /**
     * Retrieves the defined start pose from the first path segment of the active alliance.
     *
     * @return The starting {@link Pose} of the first path in the sequence.
     * @throws IndexOutOfBoundsException if no paths are defined for the active alliance.
     */
    public Pose getStartPose() {
        if (Alliance.isRed()) {
            if (redPathSegments.isEmpty()) {
                throw new IndexOutOfBoundsException("No paths defined for the Red alliance.");
            }
            return redPathSegments.get(0).firstPath().getPose(0);
        } else {
            if (bluePathSegments.isEmpty()) {
                throw new IndexOutOfBoundsException("No paths defined for the Blue alliance.");
            }
            return bluePathSegments.get(0).firstPath().getPose(0);
        }
    }

    /**
     * Adds a path segment to the Red alliance sequence.
     *
     * @param path The {@link PathChain} to add.
     */
    protected void addRedPath(PathChain path) {
        this.redPathSegments.add(path);
    }

    /**
     * Adds a path segment to the Blue alliance sequence.
     *
     * @param path The {@link PathChain} to add.
     */
    protected void addBluePath(PathChain path) {
        this.bluePathSegments.add(path);
    }
}
