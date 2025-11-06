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

@TeleOp (name = "_TeleOp_Driver_Only", group = "Main")
public class TeleOpSingleOp extends OpMode {

    private boolean isRobotCentric = false;
    private boolean isSlowMo = false;
    private boolean autoAim = false;
    private boolean isIntakeActive = false;


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
        if (gamepad1.leftStickButtonWasPressed()) {
            isRobotCentric = !isRobotCentric;
        }
        if (gamepad1.rightStickButtonWasPressed()) {
            isSlowMo = !isSlowMo;
        }

        // Auto-aim is active only while the X button is held down
        if (gamepad1.xWasPressed()) {
            autoAim = true;
        } else if (gamepad1.xWasReleased()) {
            autoAim = false;
        }

        robot.drive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                isSlowMo,
                isRobotCentric,
                autoAim);

        if (gamepad1.yWasPressed()) {
            // TODO: can use Pose offset to keep current pose
            robot.resetIMU();
        }
    }

    /**
     * Manages the shooting state machine.
     */
    private void handleShooterControls() {
        // 'A' button toggles the shooting sequence
        if (gamepad1.aWasPressed()) {
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
        if (gamepad1.dpadUpWasPressed()) {
            robot.increaseShooterRpm();
        }
        if (gamepad1.dpadDownWasPressed()) {
            robot.decreaseShooterRpm();
        }

        // Target range selection using triggers
        if (gamepad1.right_trigger > 0.2) {
            robot.setBurstFire(true);
        } else if(gamepad1.left_trigger > 0.2) {
            robot.setBurstFire(false);
        }
    }

    /**
     * Manages intake controls. Logic is improved to handle overlapping presses.
     */
    private void handleIntakeControls() {
        boolean lastIsIntakeActive = isIntakeActive;

        if (gamepad1.rightBumperWasPressed()) {
            isIntakeActive = !isIntakeActive;
        }

        if (isIntakeActive != lastIsIntakeActive) {
            if (isIntakeActive) {
                robot.startIntake();
            } else {
                robot.stopIntake();
            }
        }  else if (gamepad1.leftBumperWasPressed()) {
            robot.reverseIntake();
        } else if (gamepad1.leftBumperWasReleased()){
            // Stop intake only if neither bumper is pressed
            robot.stopIntake();
        }

        if (gamepad1.startWasPressed()){
            robot.toggleAutoIntaking();
        }
    }

    /**
     * Manages spindexer position controls.
     */
    private void handleSpindexerControls() {
        if (gamepad1.bWasPressed()) {
            robot.incrementSpindexerSlot();
        }
        if (gamepad1.dpadLeftWasPressed()) {
            robot.increaseSpindexer();
        }
        if (gamepad1.dpadRightWasPressed()) {
            robot.decreaseSpindexer();
        }
    }

    /**
     * Handles miscellaneous controls like pipeline toggling.
     */
    private void handleOtherControls() {
        if (gamepad1.backWasPressed()) {
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
