package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;

public class PathMid6Ball extends PathManager {

    public PathMid6Ball(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addRedPaths() {
        addRedPath(
        // name: Shoot1 Near, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(126.000, 115.000), new Pose(90.100, 107.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(165.000), Math.toRadians(40.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addRedPath(
        // name: Goto PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(90.100, 107.000), new Pose(95.600, 84.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(50.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(95.600, 84.500), new Pose(128.000, 84.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Shoot2 Mid, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(128.000, 84.000), new Pose(93.400, 86.200))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(45.000))
                        .applyFollowupShotSequence(ZoneDistance.MIDDLE)
                        .build()
        );
        addRedPath(
        // name: Goto PGP, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(93.400, 86.200), new Pose(95.600, 60.800))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PGP, color: #56D7BB
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(95.600, 60.800), new Pose(133.000, 59.600))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Path 7, color: #B7C997
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(133.000, 59.600), new Pose(107.000, 60.000))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }

    public void addBluePaths() {
        addBluePath(
        // name: Shoot1 Near, color: #66B85C
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(18.000, 115.000), new Pose(53.900, 107.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(15.000), Math.toRadians(140.000))
                        .applyFirstShotSequence(ZoneDistance.NEAR)
                        .build()
        );
        addBluePath(
        // name: Goto PPG, color: #9AB55D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(53.900, 107.000), new Pose(48.400, 84.500))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(130.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PPG, color: #8878CD
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.400, 84.500), new Pose(16.000, 84.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Shoot2 Mid, color: #6BCD9D
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(16.000, 84.000), new Pose(50.600, 86.200))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(135.000))
                        .applyFollowupShotSequence(ZoneDistance.MIDDLE)
                        .build()
        );
        addBluePath(
        // name: Goto PGP, color: #8DC859
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(50.600, 86.200), new Pose(48.400, 60.800))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(135.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PGP, color: #56D7BB
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(48.400, 60.800), new Pose(11.000, 59.600))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Path 7, color: #B7C997
                pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(11.000, 59.600), new Pose(37.000, 60.000))
                        )
                        .setTangentHeadingInterpolation()
                        .build()
        );
    }

}
