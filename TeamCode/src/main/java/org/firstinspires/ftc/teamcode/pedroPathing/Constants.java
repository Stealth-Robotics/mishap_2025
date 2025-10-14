package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
            .distanceUnit(DistanceUnit.MM)
            .strafePodX(strafePodOffsetMm)
            .forwardPodY(forwardPodOffsetM)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static final MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName(leftFrontDrive)
            .rightFrontMotorName(rightFrontDrive)
            .leftRearMotorName(leftBackDrive)
            .rightRearMotorName(rightBackDrive)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD) //motors keep spinning wrong way so this is a solution
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
