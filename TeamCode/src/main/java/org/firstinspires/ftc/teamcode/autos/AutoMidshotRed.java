package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathMidShot;
import org.firstinspires.ftc.teamcode.paths.PathState;

import java.util.Arrays;

@Autonomous(name = "Shoot Mid red Side", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoMidshotRed extends AutoMidshotOne {

    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        // can use limelight data if you want
        Alliance.set(Alliance.RED);
    }
}
