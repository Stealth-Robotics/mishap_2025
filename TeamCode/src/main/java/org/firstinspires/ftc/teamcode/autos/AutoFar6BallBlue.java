package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Far Blue 6 Ball (then park)", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoFar6BallBlue extends AutoFar6Ball {
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
