package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathNearAuto1 extends PathManager {

    /**
     * Constructs a new PathManager.
     *
     * @param robot The instance of the {@link RobotSystem} this manager will interact with.
     */
    public PathNearAuto1(RobotSystem robot) {
        super(robot);
    }

    public void addPaths ( ) {

    }
    public void addBluePaths ( ) {
        Follower follower = robot.getFollower();
        addBluePath(
                follower.pathBuilder()
                        .addPath(new BezierLine(new Pose(18.900, 118.200), new Pose(43.392, 107.256))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(135))
                        .addParametricCallback(0.2, robot::setReadyShoot)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(43.392, 107.256),
                                        new Pose(57.915, 90.984),
                                        new Pose(39.368, 84.335)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .addParametricCallback(1, robot::startIntake)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(39.368, 84.335), new Pose(13.500, 84.200))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(13.500, 84.200), new Pose(47.767, 115.655))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(145))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(0.9, robot::setReadyShoot)
                        .build()
        );
    }
}