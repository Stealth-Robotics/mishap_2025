package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Shoot Sushi blue Side", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
@Disabled
public class AutoSushiBlue extends AutoSushi
{
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
