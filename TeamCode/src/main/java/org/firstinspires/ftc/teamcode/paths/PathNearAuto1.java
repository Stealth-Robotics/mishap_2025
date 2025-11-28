package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
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
               pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierCurve(
                                        new Pose(126.000, 112.200),
                                        new Pose(92.900, 119.800),
                                        new Pose(89.000, 111.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(31))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
                // name: Start PPG, color: #9AB55D
               pathBuilder()
                        .addPath(
                                // Start PPG
                                new BezierCurve(
                                        new Pose(89.000, 111.000),
                                        new Pose(87.500, 90.100),
                                        new Pose(96.300, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(-180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
                // name: Intake PPG, color: #8878CD
               pathBuilder()
                        .addPath(
                                // Intake PPG
                                new BezierLine(new Pose(96.300, 84.000), new Pose(128.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
                // name: Shoot 2, color: #6BCD9D
               pathBuilder()
                        .addPath(
                                // Shoot 2
                                new BezierLine(new Pose(128.000, 84.000), new Pose(88.000, 107.700))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(31))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
                // name: Go To PGP, color: #8DC859
               pathBuilder()
                        .addPath(
                                // Go To PGP
                                new BezierCurve(
                                        new Pose(88.000, 107.700),
                                        new Pose(76.100, 71.700),
                                        new Pose(96.100, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(-180))
                        .build());
    }

    public void addBluePaths ( ) {
        Follower follower = robot.getFollower();
        addBluePath(
               pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierCurve(
                                        new Pose(18.000, 112.200),
                                        new Pose(51.100, 119.800),
                                        new Pose(55.000, 111.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(149))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
               pathBuilder()
                        .addPath(
                                // Start PPG
                                new BezierCurve(
                                        new Pose(55.000, 111.000),
                                        new Pose(56.500, 90.100),
                                        new Pose(47.700, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
               pathBuilder()
                        .addPath(
                                // Intake PPG
                                new BezierLine(new Pose(47.700, 84.000), new Pose(16.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
               pathBuilder()
                        .addPath(
                                // Shoot 2
                                new BezierLine(new Pose(16.000, 84.000), new Pose(55.500, 109.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(149))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
               pathBuilder()
                        .addPath(
                                // Go To PGP
                                new BezierCurve(
                                        new Pose(55.500, 109.500),
                                        new Pose(67.900, 71.700),
                                        new Pose(48.000, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .build());
        }
}