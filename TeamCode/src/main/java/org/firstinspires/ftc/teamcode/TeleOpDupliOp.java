package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Alliance;
import org.firstinspires.ftc.teamcode.common.FinalPose;
import org.firstinspires.ftc.teamcode.common.Motif;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

@TeleOp (name = "_TeleOp_Driver_Operator", group = "Main")
public class TeleOpDupliOp extends OpMode {

    private boolean isRobotCentric = false;
    private boolean isSlowMo = false;
    private boolean autoAim = false;

    // State machine for shooting process
    private enum ShootState { IDLE, PREPARING, READY, SHOOTING }
    private ShootState shootState = ShootState.IDLE;

    private RobotSystem robot;
    private TelemetryManager telemetryM;
    private Follower follower;

    private boolean isSpindexerInitialized = false;

    /**
     * This method is run once when the driver hits "INIT" on the Driver Station.
     * It's used for all initialization tasks.
     */
    @Override
    public void init() {
        robot = new RobotSystem(hardwareMap, telemetry);
        robot.initSpindxerSlotsEmpty();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = robot.getFollower();
        // Set the starting pose for odometry/pathing if needed

        robot.setMotifPattern(Motif.get());

        // TODO: Experemental code to see if this fixes robot drive orientation
        // This is when auto completes and the robot is facing the correct direction
        Pose finalPose = FinalPose.getPose();
        if (Math.abs(Math.toDegrees(finalPose.getHeading()) -180) < 10
                && Alliance.isBlue()){
            robot.setControlsInverted();
        }

        follower.setStartingPose(finalPose);

        telemetryM.addData("Robot Initialized", "Waiting for start...");
    }

    /**
     * This method is called repeatedly after "INIT" is pressed but before "START".
     * It's useful for tasks that need to run during the init phase, like sensor calibration.
     */
    @Override
    public void init_loop() {
        // Continue running updates for sensors and telemetry
        robot.update();

        // Initialize the spindexer, ensuring it only runs until it's ready.
        if (!isSpindexerInitialized) {
            isSpindexerInitialized = robot.doInitSpindexer();
            telemetryM.addData("Spindexer", "Initializing...");
        } else {
            telemetryM.addData("Spindexer", "Ready!");
        }

        /// You can add other init-loop tasks here, like reading AprilTag motifs.

    }

    /**
     * This method is called once when the driver hits "START".
     * It's good for any actions that need to happen right at the beginning of the match.
     */
    @Override
    public void start() {
        // Reset any runtime timers or states if necessary
        follower.startTeleopDrive(true);
    }

    /**
     * This method is called repeatedly after "START" is pressed until "STOP".
     * This is the main tele-operated loop.
     */
    @Override
    public void loop() {
        // Always update the robot's systems at the start of the loop
        robot.update();
        // Protect the robot from early start
        if (!isSpindexerInitialized) {
            telemetryM.addLine("Waiting for Spindexer JJ!!!");
            isSpindexerInitialized = robot.doInitSpindexer();
            return;
        }

        // --- Driver Controls ---
        handleDriveControls();
        handleShooterControls();
        handleIntakeControls();
        handleSpindexerControls();
        handleOtherControls();
    }

    /**
     * Groups driving-related controls for clarity.
     */
    private void handleDriveControls() {
        if (gamepad2.leftStickButtonWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }
        if (gamepad2.rightStickButtonWasPressed()) {
            isSlowMo = !isSlowMo;
        }

        // Auto-aim is active only while the X button is held down
        if (gamepad2.xWasPressed()) {
            autoAim = true;
        } else if (gamepad2.xWasReleased()) {
            autoAim = false;
        }

        robot.drive(
                -gamepad2.left_stick_y,
                -gamepad2.left_stick_x,
                -gamepad2.right_stick_x,
                isSlowMo,
                isRobotCentric,
                autoAim);

        if (gamepad2.yWasPressed()) {
            // TODO: can use Pose offset to keep current pose
            robot.resetIMU();
        }
    }

    /**
     * Manages the shooting state machine.
     */
    private void handleShooterControls() {
        // 'A' button toggles the shooting sequence
        if (gamepad2.aWasPressed()) {
            if (shootState == ShootState.IDLE) {
                shootState = ShootState.PREPARING;
                robot.setReadyShoot(); // Start preparing the shooter
            } else {
                // A quick double press will keep the shooter spinning
                shootState = ShootState.IDLE;
            }
        }

        if (shootState == ShootState.PREPARING) {
            if (robot.getShootReady()) {
                shootState = ShootState.READY;
            }
        }

        if (shootState == ShootState.READY) {
            if (robot.tryShoot()) {
                shootState = ShootState.SHOOTING; // Move to a transient state
            }
        }

        // After shooting, automatically return to idle
        if (shootState == ShootState.SHOOTING) {
            shootState = ShootState.IDLE;
        }

        // Manual RPM adjustments
        if (gamepad2.dpadUpWasPressed()) {
            robot.increaseShooterRpm();
        }
        if (gamepad2.dpadDownWasPressed()) {
            robot.decreaseShooterRpm();
        }

        // Target range selection using triggers
        if (gamepad2.left_trigger > 0.2) {
            robot.setShooterTargetRange(false); // e.g., Close Shot
        } else if (gamepad2.right_trigger > 0.2) {
            robot.setShooterTargetRange(true); // e.g., Far Shot
        }
    }

    /**
     * Manages intake controls. Logic is improved to handle overlapping presses.
     */
    private void handleIntakeControls() {
        if (gamepad2.rightBumperWasPressed()) {
            robot.startIntake();
        } else if (gamepad2.leftBumperWasPressed()) {
            robot.reverseIntake();
        } else if (gamepad2.rightBumperWasReleased() || gamepad2.leftBumperWasReleased()){
            // Stop intake only if neither bumper is pressed
            robot.stopIntake();
        }

        if (gamepad2.startWasPressed()){
            robot.toggleAutoIntaking();
        }
    }

    /**
     * Manages spindexer position controls.
     */
    private void handleSpindexerControls() {
        if (gamepad2.bWasPressed()) {
            robot.incrementSpindexerSlot();
        }
        if (gamepad2.dpadLeftWasPressed()) {
            robot.increaseSpindexer();
        }
        if (gamepad2.dpadRightWasPressed()) {
            robot.decreaseSpindexer();
        }
    }

    /**
     * Handles miscellaneous controls like pipeline toggling.
     */
    private void handleOtherControls() {
        if (gamepad2.backWasPressed()) {
            robot.resetHighLowColors();
            robot.toggleLimelightTarget();
        }
    }

    /**
     * This method is called once when "STOP" is pressed.
     * Use it to safely shut down robot systems.
     */
    @Override
    public void stop() {
        // Ensure all motors and systems are stopped gracefully.
        telemetryM.addData("Robot", "Stopped");
    }
}
