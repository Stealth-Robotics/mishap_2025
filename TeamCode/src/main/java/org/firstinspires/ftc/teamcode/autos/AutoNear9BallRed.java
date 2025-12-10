package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Near Red 9 Ball", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
@Disabled
public class AutoNear9BallRed extends AutoNear9Ball{
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.RED);
    }
}
