package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Mid 9 ball blue Side", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoMid9BallBlue extends AutoMid9Ball{
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);

    }
}
