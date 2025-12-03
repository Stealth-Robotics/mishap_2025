package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;

public class PathNear9Ball extends PathManager {

    public PathNear9Ball(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addRedPaths() {
        addRedPath(
        // name: To Shoot1 near, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126.000, 116.000), new Pose(90.000, 109.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(165.000), Math.toRadians(43.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: GOTO PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(90.000, 109.000)
                                        , new Pose(83.300, 90.100)
                                        , new Pose(96.300, 85.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(34.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(96.300, 85.000), new Pose(128.000, 84.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Shoot2 near, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(128.000, 84.000), new Pose(90.600, 107.200))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(39.000))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: GoTo PGP, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(90.600, 107.200)
                                        , new Pose(78.900, 64.500)
                                        , new Pose(96.000, 61.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(34.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PGP, color: #DA5B85
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(96.000, 61.000), new Pose(132.000, 59.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Shoot3 near, color: #B6CC89
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(132.000, 59.500)
                                        , new Pose(88.700, 57.100)
                                        , new Pose(90.000, 109.000)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
    }

    public void addBluePaths() {
        addBluePath(
        // name: To Shoot1 near, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(18.000, 116.000), new Pose(54.000, 109.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(15.000), Math.toRadians(137.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: GOTO PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(54.000, 109.000)
                                        , new Pose(60.700, 90.100)
                                        , new Pose(47.700, 85.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(146.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(47.700, 85.000), new Pose(16.000, 84.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Shoot2 near, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(16.000, 84.000), new Pose(53.400, 107.200))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(141.000))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: GoTo PGP, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(53.400, 107.200)
                                        , new Pose(65.100, 64.500)
                                        , new Pose(48.000, 61.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(146.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PGP, color: #DA5B85
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.000, 61.000), new Pose(12.000, 59.500))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Shoot3 near, color: #B6CC89
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(12.000, 59.500)
                                        , new Pose(55.300, 57.100)
                                        , new Pose(54.000, 109.000)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
    }

}
