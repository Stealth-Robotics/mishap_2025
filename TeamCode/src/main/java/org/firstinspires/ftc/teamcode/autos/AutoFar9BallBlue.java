package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Shoot Far Blue 9 Ball", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoFar9BallBlue extends AutoFar9Ball{

    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
