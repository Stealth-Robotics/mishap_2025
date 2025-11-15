package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathBallThief extends PathManager {

    public PathBallThief(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addBluePaths() {
        Follower follower = robot.getFollower();
        addBluePath(
        // name: Shoot1, color: #89D585
                follower.pathBuilder()
                        .addPath(
                                // Shoot1
                                new BezierLine(new Pose(57.000, 9.000), new Pose(57.000, 18.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .addParametricCallback(0, robot::trySelectFirstMotifSlot)
                        .addParametricCallback(0.5, ()-> {
                            robot.setShooterTargetRangeFar();
                            robot.startShooter();
                        })
                        .build()
        );
        addBluePath(
        // name: Steal Balls 1, color: #87AAA9
                follower.pathBuilder()
                        .addPath(
                                // Steal Balls 1
                                new BezierLine(new Pose(57.000, 18.000), new Pose(23.400, 19.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Steal 2, color: #979D79
                follower.pathBuilder()
                        .addPath(
                                // Steal 2
                                new BezierLine(new Pose(23.400, 19.000), new Pose(8.100, 19.000))
                        )
                        .addParametricCallback(.1, ()->follower.setMaxPower(.20))
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
        );
        addBluePath(
        // name: Steal 3, color: #D6D56C
                follower.pathBuilder()
                        .addPath(
                                // Steal 3
                                new BezierLine(new Pose(8.100, 19.000), new Pose(17.000, 19.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addParametricCallback(.1, ()->follower.setMaxPower(.50))
                        .build()
        );
        addBluePath(
        // name: Steal4, color: #8CA6D8
                follower.pathBuilder()
                        .addPath(
                                // Steal4
                                new BezierLine(new Pose(17.000, 19.000), new Pose(17.000, 8.100))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .build()
        );
        addBluePath(
        // name: Steal 5, color: #A79B78
                follower.pathBuilder()
                        .addPath(
                                // Steal 5
                                new BezierLine(new Pose(17.000, 8.100), new Pose(8.300, 8.100))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .addParametricCallback(.1, ()->follower.setMaxPower(.20))

                        .build()
        );
        addBluePath(
        // name: Shoot2, color: #C7AC68
                follower.pathBuilder()
                        .addPath(
                                // Shoot2
                                new BezierLine(new Pose(8.300, 8.100), new Pose(57.000, 17.800))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(110))
                        .addParametricCallback(0, () -> follower.setMaxPower(1))
                        // Check in on the spindexer to see if there are any unknown artifacts and try to sort them
                        .addCallback(
                                () -> !robot.isSpindexerBusy()
                                        && robot.isAnyArtifactUnknown()
                                        && robot.isHoodShootPose(),
                                robot::incrementSpindexerSlot)

                        .addParametricCallback(.8, () -> {
                            robot.trySelectFirstMotifSlot();
                            robot.startShooter();
                        })

                        .build()
        );
        addBluePath(
        // name: Park, color: #5875CB
                follower.pathBuilder()
                        .addPath(
                                // Park
                                new BezierLine(new Pose(57.000, 17.800), new Pose(45.000, 36.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .build()
        );
    }
    public void addRedPaths() {
        Follower follower = robot.getFollower();
        addRedPath(
        // name: Shoot1, color: #89D585
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57, 9), new Pose(57, 18))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))
                        .build()
        );
        addRedPath(
        // name: Steal Balls 1, color: #87AAA9
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57, 18), new Pose(23.4, 19))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .build()
        );
        addRedPath(
        // name: Steal 2, color: #979D79
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(23.4, 19), new Pose(8.1, 19))
                        )
                        .build()
        );
        addRedPath(
        // name: Steal 3, color: #D6D56C
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(8.1, 19), new Pose(17, 19))
                        )
                        .build()
        );
        addRedPath(
        // name: Steal4, color: #8CA6D8
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(17, 19), new Pose(17, 8.1))
                        )
                        .build()
        );
        addRedPath(
        // name: Steal 5, color: #A79B78
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(17, 8.1), new Pose(8.3, 8.1))
                        )
                        .build()
        );
        addRedPath(
        // name: Shoot2, color: #C7AC68
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(8.3, 8.1), new Pose(57, 17.8))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(110))
                        .build()
        );
        addRedPath(
        // name: Park, color: #5875CB
                follower.pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(57, 17.8), new Pose(45, 36))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(0))
                        .build()
        );
    }
}
