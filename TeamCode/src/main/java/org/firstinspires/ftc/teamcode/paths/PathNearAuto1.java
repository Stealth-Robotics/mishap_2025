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
                                // To Shoot 1
                                new BezierLine(new Pose(126.300, 114.100), new Pose(85.600, 104.600))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(158), Math.toRadians(37))
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 2
                                new BezierCurve(
                                        new Pose(85.600, 104.600),
                                        new Pose(86.000, 91.000),
                                        new Pose(96.300, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 3
                                new BezierLine(new Pose(96.300, 84.000), new Pose(120.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 4
                                new BezierLine(new Pose(120.000, 84.000), new Pose(85.400, 104.600))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                        .addParametricCallback(0, robot::stopIntake)
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .build()
        );
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 5
                                new BezierCurve(
                                        new Pose(85.400, 104.600),
                                        new Pose(77.500, 59.900),
                                        new Pose(98.400, 60.000)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build());
    }

    public void addBluePaths ( ) {
        Follower follower = robot.getFollower();
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierCurve(
                                        new Pose(18.300, 113.200),
                                        new Pose(51.312, 109.852),
                                        new Pose(57.300, 85.400)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(22), Math.toRadians(135))
                        .addParametricCallback(.99, robot::setReadyShoot)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 2
                                new BezierCurve(
                                        new Pose(57.300, 85.400),
                                        new Pose(59.400, 87.500),
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
                                // Path 4
                                new BezierLine(new Pose(22.000, 84.000), new Pose(57.300, 85.400))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                        .addParametricCallback(0, robot::stopIntake)
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(.99, robot::setReadyShoot)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 5
                                new BezierCurve(
                                        new Pose(57.300, 85.400),
                                        new Pose(62.800, 71.600),
                                        new Pose(47.900, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        .build());
        }
}