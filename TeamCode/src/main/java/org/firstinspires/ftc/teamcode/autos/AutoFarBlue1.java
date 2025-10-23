package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.paths.Path;
import org.firstinspires.ftc.teamcode.paths.PathFarAuto1;
import org.firstinspires.ftc.teamcode.paths.PathState;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

import java.util.Arrays;
import java.util.HashSet;

@Autonomous(name = "Far Blue 1", group = "Autonomous", preselectTeleOp = "_TeleOp Driver Only")
@Configurable
public class AutoFarBlue1 extends AutosDecode {
    // Timer for handling delays


    // Define HashSets for your action indices
    //protected final HashSet<Integer> shootIndexes = new HashSet<>(Arrays.asList(1, 7));
    //protected final HashSet<Integer> intakeIndexes = new HashSet<>(Arrays.asList(3, 4, 5));


    @Override
    protected Path initPaths() {
        shootIndexes.addAll(Arrays.asList(1, 7));
        intakeIndexes.addAll(Arrays.asList(3, 4, 5));

        return new PathFarAuto1(robot);
    }

    @Override
    protected void setAlliance() {
        // Set the specific alliance for this OpMode
        // can use limelight data if you want
        Alliance.set(Alliance.BLUE);
    }

    @Override
    protected PathState checkIndexForAction() {
        // First, call the base implementation for any common actions
        PathState baseState = super.checkIndexForAction();
        if (baseState != PathState.IDLE) {
            return baseState; // The base class is handling something
        }

        // Add any specific actions here for the auto here
        // shooting and intaking are in in the base class


        return PathState.IDLE;
    }
}
