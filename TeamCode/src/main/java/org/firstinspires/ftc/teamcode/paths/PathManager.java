package org.firstinspires.ftc.teamcode.paths;

import androidx.annotation.Nullable;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.*;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.ArrayList;
import java.util.List;

public class PathManager implements Path {
    protected Pose startPose;
    protected final List<PathChain> redPathSegments = new ArrayList<>();
    protected final List<PathChain> bluePathSegments = new ArrayList<>();
    protected int currentPath = 0;
    //protected PathBuilder builder;

    protected RobotSystem robot;

    /**
     * Constructs a new PathManager with the given start pose.
     *
     * @param robot The robot instance.
     *
     */
    public PathManager(RobotSystem robot) {
        this.robot = robot;
    }

    /**
     * Retrieves the next path segment in the sequence.
     *
     * @return The next path segment, or null if there are no more segments.
     */
    @Nullable
    public PathChain getNextSegment() {

        if (Alliance.isRed() && currentPath >= redPathSegments.size()) {
            return null;
        } else if (currentPath >= bluePathSegments.size()) {
            return null;
        }

        return Alliance.isRed() ? redPathSegments.get(currentPath++) : bluePathSegments.get(currentPath++);
    }

    /**
     * Retrieves the total number of path segments.
     * @return The number of path segments.
     */
    public int getSegmentCount() {
        return Alliance.isRed() ? redPathSegments.size() : bluePathSegments.size();
    }

    @Override
    public int getSegmentIndex() {
        return currentPath;
    }

    /**
     * Sets the start pose for the path manager.
     *
     * @param startPose The new start pose.
     */
    public void setStartPose(Pose startPose) {
        this.startPose = startPose;
        robot.getFollower().setStartingPose(startPose);

        // Set alliance based on field pose
        if (startPose.getX() > (Constants.FIELD_SIZE_X_INCHES / 2)) {
            Alliance.set(Alliance.RED);
        } else {
            Alliance.set(Alliance.BLUE);
        }
    }

    /**
     * Retrieves the start pose for the path manager.
     *
     * @return The start pose.
     */
    public Pose getStartPose() {
        return Alliance.isRed() ?
                redPathSegments.get(0).firstPath().getPose(0)
                : bluePathSegments.get(0).firstPath().getPose(0);
    }

    protected void addRedPath(PathChain path) {
        this.redPathSegments.add(path);
    }

    protected void addBluePath(PathChain path) {
        this.bluePathSegments.add(path);
    }
}
