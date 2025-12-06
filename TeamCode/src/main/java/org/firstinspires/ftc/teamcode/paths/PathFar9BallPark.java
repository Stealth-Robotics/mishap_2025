package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;

public class PathFar9BallPark extends PathManager {

    public PathFar9BallPark(RobotSystem robot) {
        super(robot);
        addPaths();
    }

    public void addPaths ( ) {
        addRedPaths();
        addBluePaths();
    }

    public void addBluePaths() {
        addBluePath(
        // name: Shoot1 Far, color: #89D585
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(57.000, 9.000), new Pose(58.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(108.000))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
        // name: GoTo GPP, color: #87AAA9
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierCurve(
                                        new Pose(58.000, 17.000)
                                        , new Pose(58.400, 34.600)
                                        , new Pose(48.000, 35.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake GPP, color: #979D79
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(48.000, 35.000), new Pose(11.000, 36.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Shoot2 Far, color: #B577AD
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierCurve(
                                        new Pose(11.000, 36.000)
                                        , new Pose(28.200, 18.500)
                                        , new Pose(54.000, 17.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0.000), Math.toRadians(108.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
        // name: GoTo PGP, color: #7BAAAC
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierCurve(
                                        new Pose(54.000, 17.000)
                                        , new Pose(58.700, 53.400)
                                        , new Pose(48.000, 59.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108.000), Math.toRadians(0.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake PGP, color: #7C8996
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(48.000, 59.000), new Pose(11.000, 60.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(0.000))
                        .applyIntakeSequence()
                        .build()
        );
        addBluePath(
        // name: Park, color: #CDBBBA
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(11.000, 60.000), new Pose(37.600, 60.000))
                        )
                        .setTangentHeadingInterpolation()
                        .applyParkSequence()
                        .build()
        );
    }

    public void addRedPaths() {
        addRedPath(
        // name: Shoot1 Far, color: #89D585
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(87.000, 9.000), new Pose(86.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(72.000))
                        .applyFirstShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
        // name: GoTo GPP, color: #87AAA9
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierCurve(
                                        new Pose(86.000, 17.000)
                                        , new Pose(85.600, 34.600)
                                        , new Pose(96.000, 35.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake GPP, color: #979D79
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(96.000, 35.000), new Pose(133.000, 36.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Shoot2 Far, color: #B577AD
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierCurve(
                                        new Pose(133.000, 36.000)
                                        , new Pose(115.800, 18.500)
                                        , new Pose(90.000, 17.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(-180.000), Math.toRadians(72.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
        // name: GoTo PGP, color: #7BAAAC
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierCurve(
                                        new Pose(90.000, 17.000)
                                        , new Pose(85.300, 53.400)
                                        , new Pose(96.000, 59.000)
                                )
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(-180.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake PGP, color: #7C8996
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(96.000, 59.000), new Pose(133.000, 60.000))
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(-180.000))
                        .applyIntakeSequence()
                        .build()
        );
        addRedPath(
        // name: Park, color: #CDBBBA
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(133.000, 60.000), new Pose(106.400, 60.000))
                        )
                        .setTangentHeadingInterpolation()
                        .applyParkSequence()
                        .build()
        );
    }

}
