package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathMidShot extends PathManager{

    public PathMidShot(RobotSystem robot){
        super(robot);
        buildPaths();
    }

    private void buildPaths()
    {
        buildBluePath();
        buildRedPath();
    }

    private void buildRedPath() {
        Follower follower = robot.getFollower();
        // Start against wall in far shoot zone
        // Move to Shoot 1
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierCurve(
                                        new Pose(126.000, 112.200),
                                        new Pose(92.600, 109.900),
                                        new Pose(90.600, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(152), Math.toRadians(50))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(0.8, ()-> {
                            robot.setShooterTargetRangeMid();
                            robot.startShooter();
                        })
                        .build());
        // Move to line 1 intake area
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 2
                                new BezierLine(new Pose(90.600, 84.000), new Pose(99.000, 84.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(50), Math.toRadians(-180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 3
                                new BezierLine(new Pose(99.000, 84.000), new Pose(128.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.1, ()->follower.setMaxPower(.20))
                        .build()
        );

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 4
                                new BezierLine(new Pose(128.000, 84.000), new Pose(90.600, 84.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(45))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        // Check in on the spindexer to see if there are any unknown artifacts and try to sort them
                        .addCallback(
                                () -> !robot.isSpindexerBusy()
                                        && robot.isAnyArtifactUnknown()
                                        && robot.isHoodShootPose(),
                                robot::incrementSpindexerSlot)
                        .addParametricCallback(.8, () -> {
                            robot.trySelectFirstMotifSlot();
                            robot.startShooter();
                        })
                        .build()
        );

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 5
                                new BezierCurve(
                                        new Pose(90.600, 84.000),
                                        new Pose(88.000, 59.000),
                                        new Pose(100.100, 60.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(-180))
                        .build()
        );
    }

    private void buildBluePath() {
        Follower follower = robot.getFollower();
        // Start against wall in far shoot zone
        // Move to Shoot 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierCurve(
                                        new Pose(18.000, 112.200),
                                        new Pose(51.400, 109.900),
                                        new Pose(53.400, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(130))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(0.8, ()-> {
                            robot.setShooterTargetRangeMid();
                            robot.startShooter();
                        })
                        .build());
        // Move to line 1 intake area
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 2
                                new BezierLine(new Pose(53.400, 84.000), new Pose(45.000, 84.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 3
                                new BezierLine(new Pose(45.000, 84.000), new Pose(16.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.1, ()->follower.setMaxPower(.20))
                        .addParametricCallback(.99, robot::stopIntake)
                        .build()
        );

        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 4
                                new BezierLine(new Pose(16.000, 84.000), new Pose(53.400, 84.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        // Check in on the spindexer to see if there are any unknown artifacts and try to sort them
                        .addCallback(
                                () -> !robot.isSpindexerBusy()
                                        && robot.isAnyArtifactUnknown()
                                        && robot.isHoodShootPose(),
                                robot::incrementSpindexerSlot)
                        .addParametricCallback(.8, () -> {
                            robot.trySelectFirstMotifSlot();
                            robot.startShooter();
                        })
                        .build()
        );

        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 5
                                new BezierCurve(
                                        new Pose(53.400, 84.000),
                                        new Pose(56.000, 59.000),
                                        new Pose(43.900, 60.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(0))
                        // get the shooter motors spinning early
                        .build()
        );
    }
}
