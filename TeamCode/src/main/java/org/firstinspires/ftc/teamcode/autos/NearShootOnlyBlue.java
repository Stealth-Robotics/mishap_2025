package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Near shoot only Blue", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
public class NearShootOnlyBlue extends NearShootOnly
{
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
