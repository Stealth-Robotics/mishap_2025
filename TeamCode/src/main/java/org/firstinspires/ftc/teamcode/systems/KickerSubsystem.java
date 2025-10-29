package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Manages the kicker mechanism of the robot.
 * This subsystem controls a servo to "kick" or interact with an object.
 */
public class KickerSubsystem {

    /** The servo motor responsible for the kicking action. */
    private final Servo kickerServo;

    /** The servo position when the kicker is in the 'up' or 'kicked' state. */
    private static final double KICKER_UP_POSITION = 0.55;

    /** The servo position when the kicker is in the 'down' or 'ready' state. */
    private static final double KICKER_DOWN_POSITION = 0.18;


    /** Number of Ms to wait for kicker transition*/
    public static final long KICK_DELAY = 250;

    private boolean isReady = false;

    private static final Timer timer = new Timer();

    /**
     * Constructs the KickerSubsystem.
     *
     * @param hardwareMap The hardware map from the OpMode, used to initialize the servo.
     */
    public KickerSubsystem(HardwareMap hardwareMap) {
        // Initialize the servo from the robot's hardware configuration.
        this.kickerServo = hardwareMap.get(Servo.class, "kicker_servo");
        // Set the kicker to its initial "ready" position upon creation.
        setReady();
    }

    /**
     * Moves the kicker to the "up" position to perform the kick action.
     */
    public void kickIt() {
        isReady = false;
        kickerServo.setPosition(KICKER_UP_POSITION);
    }

    /**
     * Moves the kicker to the "down" position, preparing it for a kick.
     * This is the default or resting state.
     */
    public void setReady() {
        if (isReady()) {
            return;
        }

        kickerServo.setPosition(KICKER_DOWN_POSITION);
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                isReady = true;
            }
        }, KICK_DELAY);
    }

    /**
     * Checks if the kicker is in the "ready" (down) position.
     *
     * @return true if the kicker servo is at or near the ready position, false otherwise.
     */
    public boolean isReady() {
        // Compare the current servo position to the target 'down' position within a tolerance.
        return isReady && kickerServo.getPosition() <= KICKER_DOWN_POSITION + 0.01;
    }
}
