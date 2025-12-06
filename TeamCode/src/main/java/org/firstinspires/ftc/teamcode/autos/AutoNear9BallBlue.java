package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Near Blue 9 Ball", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
@Disabled
public class AutoNear9BallBlue extends AutoNear9Ball {
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
