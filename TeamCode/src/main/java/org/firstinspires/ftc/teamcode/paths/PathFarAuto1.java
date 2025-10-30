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
                                new BezierLine(new Pose(56.000, 9), new Pose(61.000, 20.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(114.5)) //114.5
                        // spin up early
                        .addParametricCallback(0.1, robot::setReadyShoot)

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
                        .addParametricCallback(1, ()-> follower.setMaxPower(.3))
                        .build());
        // Move to CHOMP POS
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(42.000, 36.000), new Pose(36.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .addParametricCallback(0, robot::startIntake)
                        .addParametricCallback(.9, ()->follower.setMaxPower(.2))
                        .build()
        );

        // Artifact 1
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(36.000, 36.000), new Pose(32.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );

        // artifact 2
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(32.000, 36.000), new Pose(26.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );
        // Artifact 3 we hope
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(26.000, 36.000), new Pose(10.000, 36.000))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build()
        );

        // GOTO SHOOT 2
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(10, 36.000), new Pose(61.000, 20.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(114.5))
                        .addParametricCallback(0, ()-> {
                            robot.stopIntake();
                            follower.setMaxPower(1);
                        })
                        // get the shooter motors spinning early
                        .addParametricCallback(0.8, robot::setReadyShoot)
                        .build()
        );
        //TODO: CONINUE TO ROW 2 or the side area
        // HOME TEST:
        addBluePath(
                follower.pathBuilder()
                        .addPath(
                                // Path 8
                                new BezierLine(new Pose(61.000, 20.000), new Pose(48, 48))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build()
        );
    }

    private void buildRedPath() {
        Follower follower = robot.getFollower();
        // Start against wall in far shoot zone
        // Move to Shoot 1
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Move to shooting
                                new BezierLine(new Pose(96.000, 8.500), new Pose(84.685, 20.471))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                        // spin up early
                        .addParametricCallback(0.1, robot::setReadyShoot)
                        .build());
        // Move to row 1
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Move to get close to the balls
                                new BezierCurve(
                                        new Pose(84.685, 20.471),
                                        new Pose(87.660, 37.443),
                                        new Pose(103.700, 35.600)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(180))
                        .build());

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(84.685, 20.471),
                                        new Pose(87.660, 37.443),
                                        new Pose(103.700, 35.600)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(180))
                        .build());

        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Move to the starting of ball 1
                                new BezierLine(new Pose(103.700, 35.600), new Pose(112.300, 35.670))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build());
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                        // Move to the end of ball 3
                        new BezierLine(new Pose(112.300, 35.670), new Pose(129.300, 35.670))
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                        .build());
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Go back to the shooting position
                                new BezierCurve(
                                        new Pose(129.300, 35.670),
                                        new Pose(108.300, 13.298),
                                        new Pose(84.685, 20.067)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(65))
                        .build());
        addRedPath(
                follower.pathBuilder()
                        .addPath(
                                // Path 6
                                new BezierLine(new Pose(84.700, 20.067), new Pose(99.900, 60.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(180))
                        .build());
    }
}
