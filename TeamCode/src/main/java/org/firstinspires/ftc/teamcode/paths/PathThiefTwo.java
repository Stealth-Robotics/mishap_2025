package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathThiefTwo extends PathManager {

    public PathThiefTwo(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addRedPaths() {
        Follower follower = robot.getFollower();
        addRedPath(
        // name: Shoot1, color: #89D585
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87, 9), new Pose(87, 18))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(1, ()-> {
                            robot.setShooterTargetRangeFar();
                            robot.startShooter();
                        })
                        .build()
        );
        addRedPath(
        // name: Steal Balls 1, color: #87AAA9
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(87, 18), new Pose(119.609, 23.307))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(70), Math.toRadians(-180))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Take Balls, color: #9B5B6A
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(119.609, 23.307)
                                        , new Pose(132, 26.3)
                                        , new Pose(135.8, 8.2)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180), Math.toRadians(90))
                        .addParametricCallback(.1, ()->follower.setMaxPower(INTAKE_SPEED))
        .build()
        );
        addRedPath(
        // name: Back Shoot, color: #75A6BA
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(135.8, 8.2), new Pose(88.2, 17.9))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
        addRedPath(
        // name: Path 5, color: #9985B5
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(88.2, 17.9), new Pose(94.675, 35.051))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }
    public void addBluePaths() {
        Follower follower = robot.getFollower();
        addBluePath(
        // name: Shoot1, color: #89D585
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57, 9), new Pose(57, 18))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .build()
        );
        addBluePath(
        // name: Steal Balls 1, color: #87AAA9
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57, 18), new Pose(24.391, 23.307))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .build()
        );
        addBluePath(
        // name: Take Balls, color: #9B5B6A
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(24.391, 23.307)
                                        , new Pose(12, 26.3)
                                        , new Pose(8.2, 8.2)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                        .build()
        );
        addBluePath(
        // name: Back Shoot, color: #75A6BA
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(8.2, 8.2), new Pose(55.8, 17.9))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
        addBluePath(
        // name: Path 5, color: #9985B5
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(55.8, 17.9), new Pose(49.325, 35.051))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }
}
