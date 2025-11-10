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
        addRedPaths();
        addBluePaths();
    }

    public void addRedPaths() {
        Follower follower = robot.getFollower();
        addRedPath(
                // name: To Shoot 1, color: #66B85C
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(126, 112.2)
                                        , new Pose(92.9, 119.8)
                                        , new Pose(86.7, 108)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(31))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(.8, ()->{
                            robot.setShooterTargetRangeNear();
                            robot.startShooter();
                        })
                        .build()
        );
        addRedPath(
                // name: Start PPG, color: #9AB55D
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(86.7, 108)
                                        , new Pose(87.5, 90.1)
                                        , new Pose(96.3, 84)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(-180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
                // name: Intake PPG, color: #8878CD
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(96.3, 84), new Pose(126, 84))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addRedPath(
                // name: Shoot 2, color: #6BCD9D
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126, 84), new Pose(86.7, 108))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(31))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(.3, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(.8, robot::startShooter)
                        .build()
        );
        addRedPath(
                // name: Go To PGP, color: #8DC859
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(86.7, 108)
                                        , new Pose(76.1, 71.7)
                                        , new Pose(96.1, 59.5)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(-180))
                        .build());
    }

    public void addBluePaths ( ) {
        Follower follower = robot.getFollower();
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierCurve(
                                        new Pose(18, 112.2),
                                        new Pose(51.1, 119.8),
                                        new Pose(57.3, 108)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(149))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(.8, ()->{
                            robot.setShooterTargetRangeNear();
                            robot.startShooter();
                        })
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Move Line PPG
                                new BezierCurve(
                                        new Pose(57.3, 108),
                                        new Pose(56.5, 90.1),
                                        new Pose(47.7, 84)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Intake PPG
                                new BezierLine(new Pose(47.7, 84), new Pose(18, 84))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0.02, () -> follower.setMaxPower(0.2))
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 2
                                new BezierLine(new Pose(18, 84), new Pose(57.3, 108))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(149))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        .addParametricCallback(.3, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(.8, robot::startShooter)
                        .build()
        );
        addBluePath(
                follower.pathBuilder()
                        // Move PGP
                        .addPath(
                                new BezierCurve(
                                        new Pose(57.3, 108),
                                        new Pose(67.9, 71.7),
                                        new Pose(47.9, 59.5)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .build());
        }
}