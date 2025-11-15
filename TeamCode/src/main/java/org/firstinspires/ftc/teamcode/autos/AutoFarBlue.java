package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathFarAuto1;
import org.firstinspires.ftc.teamcode.paths.PathState;

import java.util.Arrays;

@Autonomous(name = "Shoot Far Blue Side", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
@Configurable
public class AutoFarBlue extends AutoFarOne {
    @Override
    protected void setAlliance() {
        Alliance.set(Alliance.BLUE);
    }
}
