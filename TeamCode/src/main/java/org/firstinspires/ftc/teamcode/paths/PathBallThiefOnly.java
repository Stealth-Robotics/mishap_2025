package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathBallThiefOnly extends PathManager {

    public PathBallThiefOnly(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addBluePaths() {
        addBluePath(
        // name: Shoot1, color: #89D585
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57.000, 9.000), new Pose(57.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(108.000))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
        // name: Goto Liar, color: #87AAA9
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57.000, 17.000), new Pose(26.000, 23.400))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108.000), Math.toRadians(0.000))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Take Balls, color: #9B5B6A
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(26.000, 23.400)
                                        , new Pose(10.500, 25.000)
                                        , new Pose(11.500, 11.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(90.000))
                        .applyIntakeSequence(.3)
                        .build()
        );
        addBluePath(
        // name: Shoot2, color: #75A6BA
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(11.500, 11.500)
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
                                new BezierLine(new Pose(55.000, 17.000), new Pose(55.000, 28.000))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }
    public void addRedPaths() {
        addRedPath(
        // name: Shoot1, color: #89D585
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87.000, 9.000), new Pose(87.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(72.000))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
        // name: Goto Liar, color: #87AAA9
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87.000, 17.000), new Pose(118.000, 23.400))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(-180.000))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Take Balls, color: #9B5B6A
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(118.000, 23.400)
                                        , new Pose(133.500, 25.000)
                                        , new Pose(132.500, 11.500)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(90.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Shoot2, color: #75A6BA
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(132.500, 11.500)
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
                                new BezierLine(new Pose(89.000, 17.000), new Pose(89.000, 28.000))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }
}
