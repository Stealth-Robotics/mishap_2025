package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathFarAuto1 extends PathManager{
    /**
     * Constructs a new PathManager with the given start pose.
     *
     * @param robot The robot instance.
     */
    public PathFarAuto1(RobotSystem robot) {
        super(robot);
        buildPaths();
    }

    private void buildPaths()
    {
        buildBluePath();
        buildRedPath();
    }

    private void buildBluePath() {
        Follower follower = robot.getFollower();
        // Start against wall in far shoot zone
        // Move to Shoot 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Shoot1
                                new BezierLine(new Pose(57.000, 9.000), new Pose(58.000, 18.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .addParametricCallback(0.9, robot::setReadyShoot)
                        .build());
        // Move to line 1 intake area
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // SlowIntake1
                                new BezierCurve(
                                        new Pose(58.000, 18.000),
                                        new Pose(58.400, 34.600),
                                        new Pose(48.000, 36.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // chomp3
                                new BezierLine(new Pose(48.000, 36.000), new Pose(12.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.02, ()->follower.setMaxPower(.20))
                        .build()
        );

        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Shoot2
                                new BezierCurve(
                                        new Pose(12.000, 36.000),
                                        new Pose(28.200, 18.500),
                                        new Pose(58.000, 18.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(110))
                        .addParametricCallback(0, robot::stopIntake)
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(.99, robot::setReadyShoot)
                        .build()
        );

        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 6
                                new BezierCurve(
                                        new Pose(58.000, 18.000),
                                        new Pose(64.700, 61.300),
                                        new Pose(48.000, 59.600)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        // get the shooter motors spinning early
                        .build()
        );
    }

    private void buildRedPath() {
        Follower follower = robot.getFollower();
        // Start against wall in far shoot zone
        // Move to Shoot 1
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Move to shooting
                                new BezierLine(new Pose(87.000, 9.000), new Pose(86.000, 18.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))
                        .addParametricCallback(0.9, robot::setReadyShoot)
                        .build());
        // Move to line 1 intake area
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Move to get close to the balls
                                new BezierCurve(
                                        new Pose(86.000, 18.000),
                                        new Pose(89.700, 32.100),
                                        new Pose(96.000, 36.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Move to the end of ball 3
                                new BezierLine(new Pose(96.000, 36.000), new Pose(132.000, 35.670))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.02, ()->follower.setMaxPower(.20))
                        .build()
        );

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Go back to the shooting position
                                new BezierCurve(
                                        new Pose(132.000, 35.670),
                                        new Pose(108.300, 13.300),
                                        new Pose(86.000, 18.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(69))
                        .addParametricCallback(0, robot::stopIntake)
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(.99, robot::setReadyShoot)
                        .build()
        );

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 6
                                new BezierCurve(
                                        new Pose(86.000, 18.000),
                                        new Pose(87.700, 56.800),
                                        new Pose(96.000, 60.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(180))
                        .build()
        );
    }
}
