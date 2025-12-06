package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.common.Alliance;

@Autonomous(name = "Mid 6 ball blue (Park near gate)", group = "Blue", preselectTeleOp = "_TeleOp_Driver_Operator")
public class AutoMid6BallBlue extends AutoMid6Ball {

    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        // can use limelight data if you want
        Alliance.set(Alliance.BLUE);
    }
}
