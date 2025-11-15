package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Shoot Sushi red Side", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoSushiRed extends AutoSushi
{
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.RED);
    }
}
