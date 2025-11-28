package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathFarAuto1 extends PathManager {
    /**
     * Constructs a new PathManager with the given start pose.
     *
     * @param robot The robot instance.
     */
    public PathFarAuto1(RobotSystem robot) {
        super(robot);
        buildPaths();
    }

    private void buildPaths() {
        buildBluePath();
        buildRedPath();
    }

    private void buildRedPath() {
        // Move to Shoot 1
        addRedPath(
                pathBuilder()
                        .addPath(
                                // Shoot1
                                new BezierLine(new Pose(87.000, 9.000), new Pose(86.000, 18.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build());
        // Move to line 1 intake area
        addRedPath(
                pathBuilder()
                        .addPath(
                                // Goto GPP
                                new BezierCurve(
                                        new Pose(86.000, 18.000),
                                        new Pose(85.600, 34.600),
                                        new Pose(96.000, 36.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-180))
                        .addParametricCallback(.99, robot::startIntake)
                        .build());
        // Start Chomp
        addRedPath(
                pathBuilder()
                        .addPath(
                                // Intake GPP
                                new BezierLine(new Pose(96.000, 36.000), new Pose(132.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .applyIntakeSequence()
                        .build()
        );

        addRedPath(
                pathBuilder()
                        .addPath(
                                // Shoot2
                                new BezierCurve(
                                        new Pose(132.000, 36.000),
                                        new Pose(115.800, 18.500),
                                        new Pose(86.000, 18.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(72))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );

        addRedPath(
                pathBuilder()
                        .addPath(
                                // GoTo PGP
                                new BezierCurve(
                                        new Pose(86.000, 18.000),
                                        new Pose(84.738, 62.153),
                                        new Pose(96.000, 59.600)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-180))
                        .build()
        );
    }

    private void buildBluePath() {
        // Move to Shoot 1
        addBluePath(
                pathBuilder()
                        .addPath(
                                // Shoot1
                                new BezierLine(new Pose(57.000, 9.000), new Pose(58.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build());
        // Move to line 1 intake area
        addBluePath(
                pathBuilder()
                        .addPath(
                                // SlowIntake1
                                new BezierCurve(
                                        new Pose(58.000, 17.000),
                                        new Pose(58.400, 34.600),
                                        new Pose(48.000, 35.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .addParametricCallback(.99, robot::startIntake)
                        .build());
        // Start Chomp
        addBluePath(
                pathBuilder()
                        .addPath(
                                // chomp3
                                new BezierLine(new Pose(48.000, 36.000), new Pose(11.000, 36.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .applyIntakeSequence()
                        .build()
        );

        addBluePath(
                pathBuilder()
                        .addPath(
                                // Shoot2
                                new BezierCurve(
                                        new Pose(11.000, 36.000),
                                        new Pose(28.200, 18.500),
                                        new Pose(54.000, 18.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(110))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );

        addBluePath(
                pathBuilder()
                        .addPath(
                                // Park
                                new BezierCurve(
                                        new Pose(58.000, 18.000),
                                        new Pose(64.700, 61.300),
                                        new Pose(48.000, 59.600)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .build()
        );
    }
}
