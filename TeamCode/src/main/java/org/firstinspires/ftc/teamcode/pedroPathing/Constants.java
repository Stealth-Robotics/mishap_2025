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
    public static double FIELD_SIZE_X_INCHES = 144;
    public static double FIELD_SIZE_Y_INCHES = 144;
    public static final String leftFrontDrive = "front_left_drive";
    public static final String rightFrontDrive = "front_right_drive";
    public static final String leftBackDrive = "rear_left_drive";
    public static final String rightBackDrive = "rear_right_drive";
    private static final double forwardPodOffsetMm = 82.55;
    private static final double strafePodOffsetMm = 165.1;

    private static final double forwardVelocity = 61.6;
    private static final double strafeVelocity = 50.1;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // The smaller (negative) the number the further the robot will travel under 0 power
    private static final double forwardZeroPowerAcceleration = -37.14; // -38.7776 LOW // -51.637 FULL;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    private static final double lateralZeroPowerAcceleration = -66.36; // 90.7 FULL

    public static double mass = 10.3;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(mass)
            .forwardZeroPowerAcceleration(forwardZeroPowerAcceleration)
            .lateralZeroPowerAcceleration(lateralZeroPowerAcceleration)
            ;

    public static final PinpointConstants pinpointConstants = new PinpointConstants()
            .distanceUnit(DistanceUnit.MM)
            .strafePodX(strafePodOffsetMm)
            .forwardPodY(forwardPodOffsetMm)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static final MecanumConstants mecanumConstants = new MecanumConstants()
            .leftFrontMotorName(leftFrontDrive)
            .rightFrontMotorName(rightFrontDrive)
            .leftRearMotorName(leftBackDrive)
            .rightRearMotorName(rightBackDrive)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD) //motors keep spinning wrong way so this is a solution
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(forwardVelocity)
            .yVelocity(strafeVelocity);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(pinpointConstants)
                .mecanumDrivetrain(mecanumConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
