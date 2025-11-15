package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "On Wall Near Blue", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoOnWallNearShotBlue extends AutoOnWallNearShot {
    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        // can use limelight data if you want
        Alliance.set(Alliance.BLUE);
    }
}
