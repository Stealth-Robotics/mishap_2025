package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Manages the kicker mechanism of the robot.
 * This subsystem controls a servo to "kick" or interact with an object.
 */
public class KickerSubsystem {

    /** The servo motor responsible for the kicking action. */
    private final Servo kickerServo;

    /** The servo position when the kicker is in the 'up' or 'kicked' state. */
    private static final double KICKER_KICK_POSITION = 0.508;

    /** The servo position when the kicker is in the 'down' or 'ready' state. */
    private static final double KICKER_READY_POSITION = 0.15;


    /** Number of Ms to wait for kicker transition*/
    public static final long KICK_DELAY = 500;

    private boolean isReady = false;

    //private static final Timer timer = new Timer();
    private static final ElapsedTime timer = new ElapsedTime();


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

    public void update() {
        if (isReady) {
            return;
        }

        if (timer.milliseconds() > KICK_DELAY && kickerServo.getPosition() <= KICKER_READY_POSITION + 0.01) {
            isReady = true;
        } else if (timer.milliseconds() > KICK_DELAY / 2.0) {
            kickerServo.setPosition(KICKER_READY_POSITION);
        }
    }

    /**
     * Moves the kicker to the "KICK" position to perform the kick action.
     */
    public void kickIt() {
        isReady = false;
        timer.reset();
        kickerServo.setPosition(KICKER_KICK_POSITION);
    }

    /**
     * Moves the kicker to the "READY" position, preparing it for a kick.
     * This is the default or resting state.
     */
    public void setReady() {

        kickerServo.setPosition(KICKER_READY_POSITION);
    }

    /**
     * Checks if the kicker is in the "ready" (down) position.
     *
     * @return true if the kicker servo is at or near the ready position, false otherwise.
     */
    public boolean isReady() {
        // Compare the current servo position to the target 'down' position within a tolerance.
        return isReady && kickerServo.getPosition() <= KICKER_READY_POSITION + 0.05;
    }
}
