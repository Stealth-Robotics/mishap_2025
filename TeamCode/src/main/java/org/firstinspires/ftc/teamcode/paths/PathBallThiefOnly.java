package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;

public class PathBallThiefOnly extends PathManager {

    public PathBallThiefOnly(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addRedPaths() {
        addRedPath(
        // name: Shoot1 Far, color: #89D585
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87.000, 9.000), new Pose(87.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(72.000))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
        // name: Goto PGP, color: #87AAA9
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87.000, 17.000), new Pose(121.700, 19.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PGP, color: #9B5B6A
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(121.700, 19.500)
                                        , new Pose(137.300, 20.800)
                                        , new Pose(125.000, 9.200)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(90.000))
                        .applyIntakeSequence(.15, .4)
                        .build()
        );
        addRedPath(
        // name: Shoot2 Far, color: #75A6BA
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(125.000, 9.200)
                                        , new Pose(116.200, 7.400)
                                        , new Pose(89.000, 17.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(82.000), Math.toRadians(72.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
        // name: Park, color: #9985B5
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(89.000, 17.000), new Pose(89.000, 29.000))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }

    public void addBluePaths() {
        addBluePath(
        // name: Shoot1 Far, color: #89D585
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57.000, 9.000), new Pose(57.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(108.000))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
        // name: Goto PGP, color: #87AAA9
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57.000, 17.000), new Pose(22.300, 19.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PGP, color: #9B5B6A
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(22.300, 19.500)
                                        , new Pose(6.700, 20.800)
                                        , new Pose(19.000, 9.200)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(90.000))
                        .applyIntakeSequence(.15, .4)

                        .build()
        );
        addBluePath(
        // name: Shoot2 Far, color: #75A6BA
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(19.000, 9.200)
                                        , new Pose(27.800, 7.400)
                                        , new Pose(55.000, 17.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(98.000), Math.toRadians(108.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
        // name: Park, color: #9985B5
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(55.000, 17.000), new Pose(55.000, 29.000))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }
}
