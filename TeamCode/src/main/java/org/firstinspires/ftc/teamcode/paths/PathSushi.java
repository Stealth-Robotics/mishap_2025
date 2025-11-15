package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathSushi extends PathManager {

    public PathSushi(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addBluePaths() {
        Follower follower = robot.getFollower();
        addBluePath(
        // name: To Shoot 1, color: #66B85C
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126, 112.2), new Pose(85.3, 123.1))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(14))
                        .build()
        );
    }
    public void addRedPaths() {
        Follower follower = robot.getFollower();
        addRedPath(
        // name: To Shoot 1, color: #66B85C
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(18, 112.2), new Pose(58.7, 123.1))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(166))
                        .build()
        );
    }
}
