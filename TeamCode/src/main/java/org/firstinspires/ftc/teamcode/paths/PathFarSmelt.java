package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathFarSmelt extends PathManager{
    /**
     * Constructs a new PathManager with the given start pose.
     *
     * @param robot The robot instance.
     */
    public PathFarSmelt(RobotSystem robot) {
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
                                // Shoot1
                                new BezierLine(new Pose(87.000, 9.000), new Pose(86.000, 18.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(0.5, ()-> {
                            robot.setShooterTargetRangeFar();
                            robot.startShooter();
                        })
                        .build());
        // Move to line 1 intake area
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Goto GPP
                                new BezierCurve(
                                        new Pose(86.000, 18.000),
                                        new Pose(85.600, 34.600),
                                        new Pose(96.000, 36.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Intake GPP
                                new BezierLine(new Pose(96.000, 36.000), new Pose(132.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.1, ()->follower.setMaxPower(.20))
                        .build()
        );

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Shoot2
                                new BezierCurve(
                                        new Pose(132.000, 36.000),
                                        new Pose(115.800, 18.500),
                                        new Pose(86.000, 18.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(70))
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
                                // GoTo PGP
                                new BezierLine(new Pose(86.000, 18.000), new Pose(87.400, 38.800))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(0))
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
                                // Shoot1
                                new BezierLine(new Pose(57.000, 9.000), new Pose(58.000, 18.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(0.2, ()-> {
                            robot.setShooterTargetRangeFar();
                            robot.startShooter();
                        })
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
                        .addParametricCallback(.99, robot::stopIntake)
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
                                // GoTo PGP
                                new BezierLine(new Pose(58.000, 18.000), new Pose(56.5, 38.8))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                        // get the shooter motors spinning early
                        .build()
        );
    }
}
