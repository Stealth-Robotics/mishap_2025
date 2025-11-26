package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathOnWallNear extends PathManager{
    /**
     * Constructs a new PathManager.
     *
     * @param robot The instance of the {@link RobotSystem} this manager will interact with.
     */
    public PathOnWallNear(RobotSystem robot) {
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
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Go To Motif
                                new BezierLine(new Pose(128.000, 112.000), new Pose(99.000, 112.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(123))
                        .addParametricCallback(0.5, ()-> robot.setLimelightPipeline(Pipeline.APRILTAG_MOTIF))
                        .build()
        );

        // Start against wall in far shoot zone
        // Move to Shoot 1
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierLine(new Pose(99.000, 112.000), new Pose(89.900, 112.100))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(35))
                        .addParametricCallback(0.1, ()-> {
                            robot.setLimelightPipeline(Pipeline.APRILTAG_TARGET_RED);
                            robot.trySelectFirstMotifSlot();
                        })
                        .addParametricCallback(0.8, ()-> {
                            robot.setShooterTargetRangeMid();
                            robot.startShooter();
                        })
                        .build());
        // Move to line 1 intake area
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Start PPG
                                new BezierCurve(
                                        new Pose(89.900, 112.100),
                                        new Pose(87.500, 90.100),
                                        new Pose(96.300, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(-180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Intake PPG
                                new BezierLine(new Pose(96.300, 84.000), new Pose(128.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.1, ()->follower.setMaxPower(INTAKE_SPEED))
                        .build()
        );

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Shoot 2
                                new BezierLine(new Pose(128.000, 84.000), new Pose(91.600, 108.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(36))
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
                                // Go To PGP
                                new BezierCurve(
                                        new Pose(91.600, 108.500),
                                        new Pose(76.100, 71.700),
                                        new Pose(96.000, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31), Math.toRadians(-180))
                        .build()
        );
    }

    private void buildBluePath() {
        Follower follower = robot.getFollower();
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Go To Motif
                                new BezierLine(new Pose(16.000, 112.000), new Pose(45.000, 112.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(57))
                        .addParametricCallback(0.1, ()-> robot.setLimelightPipeline(Pipeline.APRILTAG_MOTIF))
                        .build()
        );
                // Start against wall in far shoot zone
        // Move to Shoot 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierLine(new Pose(45.000, 112.000), new Pose(54.100, 112.100))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(57), Math.toRadians(145))
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
                                // Start PPG
                                new BezierCurve(
                                        new Pose(54.100, 112.100),
                                        new Pose(56.500, 90.100),
                                        new Pose(47.700, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build());
        // Start Chomp
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Intake PPG
                                new BezierLine(new Pose(47.700, 84.000), new Pose(16.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(.1, ()->follower.setMaxPower(INTAKE_SPEED))
                        .addParametricCallback(.99, robot::stopIntake)
                        .build()
        );

        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Shoot 2
                                new BezierLine(new Pose(16.000, 84.000), new Pose(52.400, 108.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(144))
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
                                // Go To PGP
                                new BezierCurve(
                                        new Pose(52.400, 108.500),
                                        new Pose(67.900, 71.700),
                                        new Pose(48.000, 59.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149), Math.toRadians(0))
                        // get the shooter motors spinning early
                        .build()
        );
    }
}

