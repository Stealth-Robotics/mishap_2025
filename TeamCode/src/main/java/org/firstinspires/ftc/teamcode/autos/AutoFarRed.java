package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;


@Autonomous(name = "Shoot Far Red Side", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoFarRed extends AutoFarOne {
    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        // can use limelight data if you want
        Alliance.set(Alliance.RED);
    }
}
