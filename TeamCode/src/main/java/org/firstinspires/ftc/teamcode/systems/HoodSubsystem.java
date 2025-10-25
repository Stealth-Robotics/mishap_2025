package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;

/**
 * The HoodSubsystem class controls the robot's hood mechanism, which is operated by a single servo.
 * The hood has two primary positions: an "intake" position (open) and a "shoot" position (closed).
 * This subsystem allows for toggling between these states and provides methods for fine-tuning
 * the servo positions.
 */
public class HoodSubsystem {

    //** Fields and Constants **//

    /** The servo object representing the hood mechanism. */
    private final Servo hoodServo;

    /** The default servo position when the hood is open for intake. */
    private static final double HOOD_OPEN_POSITION = 0.59;

    /** The default servo position when the hood is closed for shooting. */
    private static final double HOOD_CLOSED_POSITION = 0.0;

    /** The current target position for the hood's open state. Can be tuned. */
    private double openPosition = HOOD_OPEN_POSITION;

    /** The current target position for the hood's closed state. Can be tuned. */
    private double closePosition = HOOD_CLOSED_POSITION;

    /** A flag indicating if the hood is in the shooting position. True if ready to shoot. */
    private boolean isInShootPosition = true;

    //** Constructor **//

    /**
     * Initializes the HoodSubsystem.
     *
     * @param hardwareMap The HardwareMap from the OpMode, used to map the servo.
     */
    public HoodSubsystem(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hood_servo");
        // Set the hood to the shooting position by default upon initialization.
        hoodShoot();
    }

    //** Public Methods **//

    /**
     * Moves the hood to the "intake" (open) position.
     * This allows the robot to collect game elements.
     */
    public void hoodIntake() {
        isInShootPosition = false;
        hoodServo.setPosition(openPosition);
    }

    /**
     * Moves the hood to the "shoot" (closed) position.
     * A short delay is introduced before the system is considered ready to shoot,
     * allowing the servo time to move.
     */
    public void hoodShoot() {
        hoodServo.setPosition(closePosition);
        // Schedule a task to update the state after a delay, ensuring the servo has moved.
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                isInShootPosition = true;
            }
        }, 500); // 200ms delay
    }

    /**
     * Checks if the hood is in the correct position for shooting.
     *
     * @return true if the hood is in the shoot position, false otherwise.
     */
    public boolean isReadyToShoot() {
        return isInShootPosition;
    }

    /**
     * Increases the servo's open position value by a small amount for tuning.
     * The value is capped at 1.0.
     */
    public void increaseOpenPosition() {
        openPosition += 0.01;
        if (openPosition > 1.0) {
            openPosition = 1.0;
        }
    }

    /**
     * Decreases the servo's open position value by a small amount for tuning.
     * The value is capped at 0.0.
     */
    public void decrementOpenPosition() {
        openPosition -= 0.01;
        if (openPosition < 0.0) {
            openPosition = 0.0;
        }
    }

    /**
     * Increases the servo's close position value by a small amount for tuning.
     * The value is capped at 1.0.
     */
    public void increaseClosePosition() {
        closePosition += 0.01;
        if (closePosition > 1.0) {
            closePosition = 1.0;
        }
    }

    /**
     * Decreases the servo's close position value by a small amount for tuning.
     * The value is capped at 0.0.
     */
    public void decrementClosePosition() {
        closePosition -= 0.01;
        if (closePosition < 0.0) {
            closePosition = 0.0;
        }
    }
}
