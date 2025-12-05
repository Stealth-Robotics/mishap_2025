package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Mid 6 ball Red", group = "Red", preselectTeleOp = "_TeleOp_Driver_Operator")
@Disabled
public class AutoMid6BallRed extends AutoMid6Ball {

    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        // can use limelight data if you want
        Alliance.set(Alliance.RED);
    }
}
