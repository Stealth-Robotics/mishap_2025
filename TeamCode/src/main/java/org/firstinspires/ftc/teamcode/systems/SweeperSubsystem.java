package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Represents the sweeper subsystem of the robot, which is responsible for intake and outtake actions.
 * This class manages two continuous rotation (CR) servos that act as the sweeper mechanism.
 */
@Configurable
public class SweeperSubsystem {

    public static double MAX_SWEEPER_SPEED = 1.0;

    public double currentPower = MAX_SWEEPER_SPEED;


    /** The left CR servo for the sweeper mechanism. */
    private final CRServo leftSweeper;

    /** The right CR servo for the sweeper mechanism. */
    private final CRServo rightSweeper;

    /**
     * Constructs a new SweeperSubsystem.
     *
     * @param hardwareMap The hardware map from the robot's configuration to initialize devices.
     */
    public SweeperSubsystem(HardwareMap hardwareMap) {
        leftSweeper = hardwareMap.get(CRServo.class, "left_sweeper_servo");
        rightSweeper = hardwareMap.get(CRServo.class, "right_sweeper_servo");

        // Reverse the direction of the right sweeper so that both sweepers rotate inwards or outwards together.
        rightSweeper.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /**
     * Activates the intake by setting the power of the sweeper servos to their maximum forward speed.
     */
    public void startIntake() {
        leftSweeper.setPower(currentPower);
        rightSweeper.setPower(currentPower);
    }

    /**
     * Stops the sweeper mechanism by setting the power of the servos to zero.
     */
    public void stopIntake() {
        leftSweeper.setPower(0.0);
        rightSweeper.setPower(0.0);
    }

    /**
     * Reverses the direction of the sweeper mechanism for outtake or to clear a jam.
     */
    public void reverseIntake() {
        leftSweeper.setPower(-currentPower);
        rightSweeper.setPower(-currentPower);
    }

    /**
     * Adjusts the sweeper power (for testing speeds)
     * @param power .01 to 1 power settings
     */
    public void setPower(double power) {
        this.currentPower = power;
    }

    /**
     * Increases the sweeper power by 10%
     */
    public void increasePercentPower() {
        if (currentPower < MAX_SWEEPER_SPEED) {
            currentPower += 0.1;
        }
    }

    /**
     * Decreases the sweeper power by 10%
     */
    public void decreasePercentPower() {
        if (currentPower > -1) {
            currentPower -= 0.1;
        }
    }

    /**
     * Gets the current sweeper power
     * @return the current sweeper power
     */
    public double getCurrentPower() {
        return currentPower;
    }
}
