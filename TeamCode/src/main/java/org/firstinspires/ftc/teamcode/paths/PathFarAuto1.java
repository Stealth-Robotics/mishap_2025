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
                                new BezierLine(new Pose(56.000, 11), new Pose(61.000, 20.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                        .addParametricCallback(0.5, ()->robot.setReadyShoot())
                        //.setTimeoutConstraint(1000)
                        .build());
        // Move to line 1 intake area
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // SlowIntake1
                                new BezierCurve(
                                        new Pose(61.000, 20.000),
                                        new Pose(66.496, 36.000),
                                        new Pose(42.000, 36.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(0))
                        .addParametricCallback(.9, ()-> follower.setMaxPower(.2))
                        .build());
        // Move to CHOMP POS
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(42.000, 36.000), new Pose(36.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0, ()->robot.startIntake())
                        .build()
        );

        // COMP 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(36.000, 36.000), new Pose(32.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );

        // CHOMP 2
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(32.000, 36.000), new Pose(26.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0, ()->robot.startIntake())
                        .build()
        );
        // CHOMP 3
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(26.000, 36.000), new Pose(18.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0, ()->robot.startIntake())

                        .build()
        );

        // GOTO SHOOT 2
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(18.00, 36.000), new Pose(61.000, 20.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(115))
                        .addParametricCallback(0, ()->{
                            follower.setMaxPower(1);
                            robot.setReadyShoot();
                        } )

                        //.addParametricCallback(.5, ()->robot.setReadyShoot())
                        //.setTimeoutConstraint(1000)
                        .build()
        );

        // HOME TEST:
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 8
                                new BezierLine(new Pose(61.000, 20.000), new Pose(56.000, 12.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(90))
                        .build()
        );
    }

    private void buildRedPath()
    {
        //TODO: IMPLEMENT
    }

}
