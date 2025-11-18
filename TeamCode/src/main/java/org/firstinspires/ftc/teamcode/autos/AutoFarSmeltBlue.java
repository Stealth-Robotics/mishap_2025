package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Smelt Far Blue Side", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoFarSmeltBlue extends AutoFarSmelt {
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
