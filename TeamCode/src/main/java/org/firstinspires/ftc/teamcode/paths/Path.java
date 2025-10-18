package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public interface Path {

    PathChain getNextSegment();

    void setStartPose(Pose startPose);

    Pose getStartPose();

    int getSegmentCount();

}