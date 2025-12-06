package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Near Shoot Only Red", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoNearShootOnlyRed extends AutoNearShootOnly
{
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.RED);
    }
}
