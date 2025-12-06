package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@SuppressWarnings("unused")
@Autonomous(name="Ball Thief Blue", group="Blue", preselectTeleOp="_TeleOp_Driver_Operator")
@Disabled
public class AutoBallThiefBlue extends AutoBallThief{

    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
