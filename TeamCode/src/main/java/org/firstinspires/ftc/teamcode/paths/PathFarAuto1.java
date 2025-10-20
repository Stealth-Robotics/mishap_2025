package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathFarAuto1 extends PathManager{
    /**
     * Constructs a new PathManager with the given start pose.
     *
     * @param robot The robot instance.
     */
    public PathFarAuto1(RobotSystem robot) {
        super(robot);
        buildPaths();

    }

    private void buildPaths()
    {
        buildBluePath();
        buildRedPath();
    }

    private void buildBluePath() {
        Follower follower = robot.getFollower();
        // Start against wall in far shoot zone
        // Move to Shoot 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(56.000, 8.500), new Pose(61.000, 20.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                        .addParametricCallback(0.5, ()->robot.setReadyShoot())
                        .build());
        // Move to line 1 intake area
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(61.000, 20.000),
                                        new Pose(66.496, 36.000),
                                        new Pose(42.000, 36.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                        .addParametricCallback(1, ()->robot.startIntake())
                        .build());
        // Move to CHOMP POS
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(42.000, 36.000), new Pose(36.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .addParametricCallback(0, ()-> follower.setMaxPower(.2))
                        .build()
        );

        // COMP 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(36.000, 36.000), new Pose(32.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .addParametricCallback(0, ()->robot.startIntake())

                        // TODO ADD CHECK for color sensor could do dynamic path build
                        .addParametricCallback(1, ()->robot.stopIntake())
                        .build()
        );

        // CHOMP 2
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(32.000, 36.000), new Pose(26.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .addParametricCallback(0, ()->robot.startIntake())

                        // TODO ADD CHECK for color sensor could do dynamic path build
                        .addParametricCallback(1, ()->robot.stopIntake())
                        .build()
        );
        // CHOMP 3
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(26.000, 36.000), new Pose(20.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .addParametricCallback(0, ()->robot.startIntake())

                        // TODO ADD CHECK for color sensor could do dynamic path build
                        .addParametricCallback(1, ()->robot.stopIntake())
                        .build()
        );

        // GOTO SHOOT 2
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(20.000, 36.000), new Pose(61.000, 20.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(115))
                        .addParametricCallback(0, ()->follower.setMaxPower(1))
                        .build()
        );
    }

    private void buildRedPath()
    {
        //TODO: IMPLEMENT
    }

}
