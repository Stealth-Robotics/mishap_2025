package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Shoot Far Blue 6 Ball", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoFarBlue extends AutoFar6Ball {
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
