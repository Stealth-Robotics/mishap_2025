package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

import java.util.ArrayList;
import java.util.List;

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
        addRedPaths();
    }

    public void addRedPaths() {
        Follower follower = robot.getFollower();
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(126.000, 112.200),
                                        new Pose(92.900, 119.800),
                                        new Pose(86.700, 108.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(158), Math.toRadians(31))
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(86.700, 108.000),
                                        new Pose(87.500, 90.100),
                                        new Pose(96.300, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(96.300, 84.000), new Pose(122.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(122.000, 84.000), new Pose(86.700, 108.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(31))
                        .addParametricCallback(0, robot::stopIntake)
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(86.700, 108.000),
                                        new Pose(76.100, 71.700),
                                        new Pose(96.100, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(180))
                        .build());
    }

    public void addBluePaths ( ) {
        Follower follower = robot.getFollower();
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(18.000, 112.200),
                                        new Pose(51.100, 119.800),
                                        new Pose(57.300, 108.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(149))
                        .addParametricCallback(.99, robot::setReadyShoot)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(57.300, 108.000),
                                        new Pose(56.500, 90.100),
                                        new Pose(47.700, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(47.700, 84.000), new Pose(22.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(22.000, 84.000), new Pose(57.300, 108.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(149))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(.99, robot::setReadyShoot)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(57.300, 108.000),
                                        new Pose(67.900, 71.700),
                                        new Pose(47.900, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .build());
        }
}