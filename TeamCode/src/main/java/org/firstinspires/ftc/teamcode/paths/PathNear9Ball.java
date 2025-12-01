package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathNear9Ball extends PathManager {

    public PathNear9Ball(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addBluePaths() {
        addBluePath(
        // name: To Shoot 1, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(18.000, 112.200), new Pose(54.000, 110.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28.000), Math.toRadians(146.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: Start PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(54.000, 110.000)
                                        , new Pose(57.100, 90.200)
                                        , new Pose(47.700, 85.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(146.000), Math.toRadians(0.000))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(47.700, 85.000), new Pose(16.000, 84.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .applyIntakeSequence(.15, .25)
                        .build()
        );
        addBluePath(
        // name: Shoot 2, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(16.000, 84.000), new Pose(54.000, 109.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(146.000))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: Go To PGP, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(54.000, 109.500)
                                        , new Pose(66.100, 67.300)
                                        , new Pose(48.000, 61.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(146.000), Math.toRadians(0.000))
                        .addParametricCallback(.99, robot::startIntake)
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
        // name: Shoot3, color: #B6CC89
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(12.000, 59.500)
                                        , new Pose(52.900, 56.400)
                                        , new Pose(55.000, 107.500)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
    }
    public void addRedPaths() {
        addRedPath(
        // name: To Shoot 1, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126.000, 112.200), new Pose(89.000, 111.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(152.000), Math.toRadians(34.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: Start PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(89.000, 111.000)
                                        , new Pose(86.900, 90.200)
                                        , new Pose(96.300, 85.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(34.000), Math.toRadians(-180.000))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(96.300, 85.000), new Pose(128.000, 84.000))
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
                                new BezierLine(new Pose(128.000, 84.000), new Pose(89.000, 109.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(34.000))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: Go To PGP, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(89.000, 109.500)
                                        , new Pose(77.900, 67.300)
                                        , new Pose(96.000, 61.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(34.000), Math.toRadians(-180.000))
                        .addParametricCallback(.99, robot::startIntake)
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
        // name: Shoot3, color: #B6CC89
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(132.000, 59.500)
                                        , new Pose(91.100, 56.400)
                                        , new Pose(89.000, 107.500)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
    }
}
