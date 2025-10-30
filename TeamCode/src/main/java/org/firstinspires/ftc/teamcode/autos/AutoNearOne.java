package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathNearAuto1;

import java.util.Arrays;
@Autonomous(name = "Shoot Near either Side", group = "Autonomous", preselectTeleOp = "_TeleOp_Driver_Only")
@Configurable
public class AutoNearOne extends AutosDecode {

    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 4));
        intakeIndexes.addAll(Arrays.asList(3));
        return new PathNearAuto1(robot);
    }

    @Override
    protected void setSpindexerSlots() {
        robot.initSpindxerSlotsEmpty();
    }
}
