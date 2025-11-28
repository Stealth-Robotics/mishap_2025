package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathOnWallNear extends PathManager{
    /**
     * Designed for squaring the bot against the arttifact sorting wall facing out to the
     * oppposing team
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

        addRedPath(
               pathBuilder()
                        .addPath(
                                // Go To Motif
                                new BezierLine(new Pose(128.000, 112.000), new Pose(99.000, 112.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(123))
                        .addParametricCallback(0.1, ()-> robot.setLimelightPipeline(Pipeline.APRILTAG_MOTIF))
                        .build()
        );

        // Start against wall in far shoot zone
        // Move to Shoot 1
        addRedPath(
               pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierLine(new Pose(99.000, 112.000), new Pose(89.900, 112.100))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(35))
                        .addParametricCallback(0.1, ()-> robot.setLimelightPipeline(Pipeline.APRILTAG_TARGET_RED))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build());
        // Move to line 1 intake area
        addRedPath(
               pathBuilder()
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
               pathBuilder()
                        .addPath(
                                // Shoot 2
                                new BezierLine(new Pose(128.000, 84.000), new Pose(91.600, 108.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(36))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );

        addRedPath(
               pathBuilder()
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

        addBluePath(
               pathBuilder()
                        .addPath(
                                // Go To Motif
                                new BezierLine(new Pose(16.000, 112.000), new Pose(45.000, 112.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(57))
                        .addParametricCallback(0.1, ()-> robot.setLimelightPipeline(Pipeline.APRILTAG_MOTIF))
                        .build()
        );

        // Move to Shoot 1
        addBluePath(
               pathBuilder()
                        .addPath(
                                // To Shoot 1
                                new BezierLine(new Pose(45.000, 112.000), new Pose(54.100, 112.100))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(57), Math.toRadians(145))
                        .addParametricCallback(0.1, ()-> {robot.setLimelightPipeline(Pipeline.APRILTAG_TARGET_BLUE);})
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build());
        // Move to line 1 intake area
        addBluePath(
               pathBuilder()
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
               pathBuilder()
                        .addPath(
                                // Intake PPG
                                new BezierLine(new Pose(47.700, 84.000), new Pose(16.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .applyIntakeSequence()
                        .build()
        );

        addBluePath(
               pathBuilder()
                        .addPath(
                                // Shoot 2
                                new BezierLine(new Pose(16.000, 84.000), new Pose(52.400, 108.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(144))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );

        addBluePath(
               pathBuilder()
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

