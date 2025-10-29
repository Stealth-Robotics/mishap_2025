package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.geometry.Pose;

public class FinalPose {
    private static Pose finalPose = new Pose(0, 0, 0);

    public static void setPose(Pose pose) {
        finalPose = pose;
    }

    public static Pose getPose() {
        return finalPose;
    }
}
