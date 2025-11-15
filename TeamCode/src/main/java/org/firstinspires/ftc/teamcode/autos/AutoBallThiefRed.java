package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name="Ball Thief Red", group="Red", preselectTeleOp="_TeleOp_Driver_Operator")
public class AutoBallThiefRed extends AutoBallThief{

    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.RED);
    }
}
