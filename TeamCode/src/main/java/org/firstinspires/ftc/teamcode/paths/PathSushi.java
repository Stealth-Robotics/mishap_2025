package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

/**
 * simple Near Auto for bots like Sushi that start far and move near
 */
public class PathSushi extends PathManager {

    public PathSushi(RobotSystem robot) {
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
                                new BezierLine(new Pose(18.000, 112.200), new Pose(56.000, 123.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(28.000), Math.toRadians(166.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: Park, color: #D7DA66
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(56.000, 123.000), new Pose(55.000, 134.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(166.000), Math.toRadians(180.000))
                        .build()
        );
    }
    public void addRedPaths() {
        addRedPath(
        // name: To Shoot 1, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126.000, 112.200), new Pose(88.000, 123.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(152.000), Math.toRadians(14.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: PARK, color: #D7DA66
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(88.000, 123.000), new Pose(89.000, 134.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(14.000), Math.toRadians(0.000))
                        .build()
        );
    }
}
