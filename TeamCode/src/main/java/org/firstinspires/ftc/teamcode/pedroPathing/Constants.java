package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static final String leftFrontDrive = "front_left_drive";
    public static final String rightFrontDrive = "front_right_drive";
    public static final String leftBackDrive = "rear_left_drive";
    public static final String rightBackDrive = "rear_right_drive";
    private static final double forwardPodOffsetM = -76;
    private static final double strafePodOffsetMm = 161;

    public static double mass = 10.1;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(mass);

    public static final PinpointConstants pinpointConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.MM);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
