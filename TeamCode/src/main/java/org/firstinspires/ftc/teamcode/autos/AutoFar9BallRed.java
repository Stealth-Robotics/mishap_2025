package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Far Red 9 Ball (shoots 6)", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoFar9BallRed extends AutoFar9Ball{

    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.RED);
    }
}
