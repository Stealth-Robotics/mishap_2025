package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.ZoneDistance;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.firstinspires.ftc.teamcode.paths.util.PathManager;

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
        // name: Goto PGP, color: #7BAAAC
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(54.000, 17.000), new Pose(15.000, 36.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108.000), Math.toRadians(75.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addBluePath(
        // name: Intake take, color: #CDCC98
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(15.000, 36.000), new Pose(12.000, 11.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(75.000), Math.toRadians(90.000))
                        .applyIntakeSequence(.2, .4)
                        .build()
        );
        addBluePath(
        // name: Shoot3 Far, color: #BC8B56
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(12.000, 11.000), new Pose(54.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(108.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addBluePath(
        // name: Park, color: #9C8D76
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(54.000, 17.000), new Pose(40.000, 30.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(108.000), Math.toRadians(90.000))
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
        // name: Goto PGP, color: #7BAAAC
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(90.000, 17.000), new Pose(129.000, 36.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(105.000))
                        .addParametricCallback(.9, robot::startIntake)
                        .build()
        );
        addRedPath(
        // name: Intake take, color: #CDCC98
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(129.000, 36.000), new Pose(132.000, 11.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(105.000), Math.toRadians(90.000))
                        .applyIntakeSequence(.2, .4)
                        .build()
        );
        addRedPath(
        // name: Shoot3 Far, color: #BC8B56
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(132.000, 11.000), new Pose(90.000, 17.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90.000), Math.toRadians(72.000))
                        .applyFollowupShotSequence(ZoneDistance.FAR)
                        .build()
        );
        addRedPath(
        // name: Park, color: #9C8D76
                pathBuilder()
                        .setGlobalDeceleration()
                        .addPath(
                                new BezierLine(new Pose(90.000, 17.000), new Pose(104.000, 30.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(72.000), Math.toRadians(90.000))
                        .applyParkSequence()
                        .build()
        );
    }

}
