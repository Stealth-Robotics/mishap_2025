package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;

public class PathNear6BallLeverPark extends PathManager {

    public PathNear6BallLeverPark(RobotSystem robot) {
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
                                new BezierLine(new Pose(18.000, 115.000), new Pose(55.000, 111.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(15.000), Math.toRadians(149.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: Start PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(55.000, 111.000)
                                        , new Pose(57.300, 91.200)
                                        , new Pose(47.700, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(149.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(47.700, 84.000), new Pose(16.000, 84.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Shoot 2, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(16.000, 84.000), new Pose(55.000, 109.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(149.000))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: ParkNearLever, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(55.000, 109.500)
                                        , new Pose(58.000, 74.000)
                                        , new Pose(24.000, 74.000)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }

    public void addRedPaths() {
        addRedPath(
        // name: To Shoot 1, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126.000, 115.000), new Pose(89.000, 111.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(165.000), Math.toRadians(31.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: Start PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(89.000, 111.000)
                                        , new Pose(86.700, 91.200)
                                        , new Pose(96.300, 84.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(31.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(96.300, 84.000), new Pose(128.000, 84.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Shoot 2, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(128.000, 84.000), new Pose(89.000, 109.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(31.000))
                        .applyFollowupShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: ParkNearLever, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(89.000, 109.500)
                                        , new Pose(86.000, 74.000)
                                        , new Pose(120.000, 74.000)
                                )
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }

}
