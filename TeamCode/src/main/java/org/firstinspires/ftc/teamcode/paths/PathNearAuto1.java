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
        addPaths();
    }

    public void addPaths ( ) {
        addBluePaths();

    }
    public void addBluePaths ( ) {
        Follower follower = robot.getFollower();
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierLine(new Pose(17.700, 114.100), new Pose(58.400, 104.600))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(22), Math.toRadians(143))
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 2
                                new BezierCurve(
                                        new Pose(58.400, 104.600),
                                        new Pose(58.000, 91.000),
                                        new Pose(47.700, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 3
                                new BezierLine(new Pose(47.700, 84.000), new Pose(24.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 4
                                new BezierLine(new Pose(24.000, 84.000), new Pose(58.600, 104.600))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140))
                        .addParametricCallback(0, robot::stopIntake)
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 5
                                new BezierCurve(
                                        new Pose(58.600, 104.600),
                                        new Pose(66.500, 59.900),
                                        new Pose(45.600, 60.000)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build());
        }
}