package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * The Path interface defines the contract for a path that is made up of multiple segments.
 * It provides methods to retrieve path segments, manage the starting position, and query
 * the state of the path's progression.
 */
public interface Path {

    /**
     * Retrieves the next segment in the path sequence.
     *
     * @return The next PathChain segment to be processed.
     */
    PathChain getNextSegment();

    /**
     * Sets the starting pose for the beginning of the path.
     *
     * @param startPose The Pose object representing the starting coordinates and heading.
     */
    void setStartPose(Pose startPose);

    /**
     * Gets the currently configured starting pose for the path.
     *
     * @return The Pose object representing the path's start.
     */
    Pose getStartPose();

    /**
     * Returns the total number of segments that make up the entire path.
     *
     * @return An integer representing the total segment count.
     */
    int getSegmentCount();

    /**
     * Returns the index of the current segment being processed.
     *
     * @return An integer for the current segment's index.
     */
    int getSegmentIndex();
}
