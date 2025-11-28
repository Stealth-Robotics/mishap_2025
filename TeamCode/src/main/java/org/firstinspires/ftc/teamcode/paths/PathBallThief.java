package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class PathBallThief extends PathManager {

    public PathBallThief(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths() {
        addRedPaths();
        addBluePaths();
    }

    public void addBluePaths() {
        Follower follower = robot.getFollower();
        addBluePath(
                // name: Shoot1, color: #89D585
                pathBuilder()
                        .addPath(
                                // Shoot1
                                new BezierLine(new Pose(57.000, 9.000), new Pose(58.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(108))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
                // name: GoTo GPP, color: #87AAA9
                pathBuilder()
                        .addPath(
                                // GoTo GPP
                                new BezierCurve(
                                        new Pose(58.000, 17.000),
                                        new Pose(58.400, 34.600),
                                        new Pose(47.500, 35.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(0))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addBluePath(
                // name: Intake GPP, color: #979D79
                pathBuilder()
                        .addPath(
                                // Intake GPP
                                new BezierLine(new Pose(47.500, 35.000), new Pose(12.000, 36.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
                pathBuilder()
                        .addPath(
                                // Shoot2
                                new BezierCurve(
                                        new Pose(12.000, 36.000),
                                        new Pose(28.200, 18.500),
                                        new Pose(55.000, 17.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(108))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );

        addBluePath(
                pathBuilder()
                        .addPath(
                                // GoTo Wall
                                new BezierLine(new Pose(55.000, 17.000), new Pose(12.000, 33.400))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(80))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addBluePath(
                pathBuilder()
                        .addPath(
                                // Steal
                                new BezierLine(new Pose(12.000, 33.400), new Pose(12.000, 12.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(80), Math.toRadians(95))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
                pathBuilder()
                        .addPath(
                                // Shoot3
                                new BezierLine(new Pose(12.000, 12.000), new Pose(55.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(95), Math.toRadians(108))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
                // name: Park, color: #5875CB
                pathBuilder()
                        .addPath(
                                // Park
                                new BezierLine(new Pose(55.000, 17.000), new Pose(55.000, 30.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108), Math.toRadians(-180))
                        .build()
        );
    }

    public void addRedPaths() {

        addRedPath(
                // name: Shoot1, color: #89D585
                pathBuilder()
                        .addPath(
                                // Shoot1
                                new BezierLine(new Pose(87.000, 9.000), new Pose(86.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(72))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
                // name: GoTo GPP, color: #87AAA9
                pathBuilder()
                        .addPath(
                                // GoTo GPP
                                new BezierCurve(
                                        new Pose(86.000, 17.000),
                                        new Pose(85.600, 34.600),
                                        new Pose(96.500, 35.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(-180))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addRedPath(
                // name: Intake GPP, color: #979D79
                pathBuilder()
                        .addPath(
                                // Intake GPP
                                new BezierLine(new Pose(96.500, 35.000), new Pose(132.000, 36.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
                // name: Shoot2, color: #B577AD
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(132.000, 36.000)
                                        , new Pose(115.800, 18.500)
                                        , new Pose(89.000, 17.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(72.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
                // name: GoTo Wall, color: #7BAAAC
                pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        new Pose(89.000, 17.000)
                                        , new Pose(98.000, 34.000)
                                        , new Pose(132.000, 33.400)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(100.000))
                        .addParametricCallback(.99, robot::startIntake)
                        .build()
        );
        addRedPath(
                // name: Steal, color: #5CCB7B
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(132.000, 33.400), new Pose(132.000, 12.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(100.000), Math.toRadians(85.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
                // name: Shoot3, color: #865D88
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(132.000, 12.000), new Pose(89.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(85.000), Math.toRadians(72.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
                // name: Park, color: #C66BCC
                pathBuilder()
                        .addPath(
                                // Park
                                new BezierLine(new Pose(89.000, 17.000), new Pose(89.000, 30.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72), Math.toRadians(0))
                        .build()
        );
    }
}
