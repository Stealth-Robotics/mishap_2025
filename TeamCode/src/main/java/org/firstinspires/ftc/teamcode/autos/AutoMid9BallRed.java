package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Mid 9 ball red (shoot 6)", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoMid9BallRed extends AutoMid9Ball {
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.RED);
    }
}
