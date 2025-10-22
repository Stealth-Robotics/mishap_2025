package org.firstinspires.ftc.teamcode.systems;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Motif;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Container of all subsystems for the robot and wrapper for all controls
 */
@Configurable
public class RobotSystem {

    public static double SLOW_MODE_MULTIPLIER = 0.3;

    // This is for auto aim functionality
    private final static PIDFCoefficients headingCoefficients
            = new PIDFCoefficients(0.018, 0.0, 0.001, 0.02);

    public static final double MAX_ROTATION_POWER = 0.80;

   public static final PIDFController headingController
           = new PIDFController(headingCoefficients);
    private final SweeperSubsystem sweeperSys;
    private final HoodSubsystem hoodSys;
    private final KickerSubsystem kickerSys;
    private final ShooterSubsystem shooterSys;
    private final SpindexerSubsystem spindexerSys;
    private final LimelightSubsystem limelightSys;
    private boolean isIntaking = false;
    private final Timer timer = new Timer();
    private volatile boolean isShootReady = false;
    private final Follower follower;
    private final TelemetryManager telemetryM;
    private final Telemetry telemetry;
    private boolean intakeStopping = false;

    private Motif motif = Motif.UNKNOWN;

    public RobotSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.sweeperSys = new SweeperSubsystem(hardwareMap);
        this.hoodSys = new HoodSubsystem(hardwareMap);
        this.kickerSys = new KickerSubsystem(hardwareMap);
        this.shooterSys = new ShooterSubsystem(hardwareMap);
        this.spindexerSys = new SpindexerSubsystem(hardwareMap);
        this.follower = Constants.createFollower(hardwareMap);
        this.limelightSys = new LimelightSubsystem(hardwareMap);
        this.telemetry = telemetry;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void update() {
        follower.update();
        limelightSys.update();
        shooterSys.update();
        this.isShootReady = shooterSys.isReadyToShoot()
                && hoodSys.shootReady()
                && kickerSys.isKickReady()
                && spindexerSys.isReady();

        // Robot Telemetry shown on screen
        telemetryM.addData("Shooter RPM", shooterSys.getCurrentRpm());
        telemetryM.addData("Target RPM", shooterSys.getTargetRpm());
        //telemetryM.addData("Spindexer ticks", spindexerSys.getPosition());
        telemetryM.addData("Switch Pressed:", spindexerSys.isIndexSwitchPressed());
        telemetryM.addData("ShootReady:", shooterSys.isReadyToShoot());
        //telemetryM.addData("HoodReady:", hoodSys.shootReady());
        //telemetryM.addData("KickerReady:", kickerSys.isKickReady());
        //telemetryM.addData("SpindexerReady:", spindexerSys.isReady());
//        telemetryM.addData("IsIntaking:", isIntaking);
        telemetryM.update(telemetry);
    }

    /**
     * Controls robot movement and wraps follower.setTeleOpDrive
     * @param y - forward/backward
     * @param x - left/right
     * @param z - rotation
     * @param robotCentric - true if robot centric
     * @param slowMo - true if slow mo
     * @param autoAim - true if auto aim
     */
    public void drive(
            double y,
            double x,
            double z,
            boolean robotCentric,
            boolean slowMo,
            boolean autoAim) {

        double slowMoFactor = slowMo ? SLOW_MODE_MULTIPLIER : 1;
        double turn = z * slowMoFactor;

        if (autoAim) {
            limelightSys.getLastResult(); // force a new entry
            Pose llPose = limelightSys.getAverageTxTy(10);
            if (llPose != null) {
                double output = getScaledTxOutput(llPose.getX(), 1);
                if (output != 0) {
                    // rotation power is swapped
                    turn = -output;
                }
            }
        }

        follower.setTeleOpDrive(
                y * slowMoFactor,
                x * slowMoFactor,
                turn,
                robotCentric);
        // DO a double update
        follower.update();
    }

    public void setMotif(Motif motif) {
        this.motif = motif;
    }


    /**
     * Returns true if the robot is ready to shoot
     * @return - true if ready to shoot
     */
    public boolean getShootReady() {
        return this.isShootReady;
    }

    public Follower getFollower() {
        return follower;
    }

    public void startIntake()
    {
        isShootReady = false;
        isIntaking = true;
        shooterSys.stop();
        kickerSys.kickReady();
        hoodSys.hoodIntake();
        sweeperSys.startIntake();
        spindexerSys.setFloat();
    }

    public void reverseIntake(){
        isShootReady = false;
        isIntaking = true;
        kickerSys.kickReady();
        hoodSys.hoodIntake();
        sweeperSys.reverseIntake();
        spindexerSys.setFloat();
    }

    public void stopIntake(){
        if (intakeStopping) {
            return;
        }

        intakeStopping = true;
        hoodSys.hoodShoot();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                sweeperSys.stopIntake();
                spindexerSys.setBrake();
                // should check for ball in slot
                spindexerSys.advanceOneSlot();
                isIntaking = false;
                intakeStopping = false;
            }
        }, 300);
    }

    public void setReadyShoot()
    {
        if (!isIntaking && !isShootReady) {
            kickerSys.kickReady();
            hoodSys.hoodShoot();
            shooterSys.runShooter();
        }
    }

    public boolean isIntakeRunning() {
        return this.isIntaking;
    }

    public boolean shoot() {
        if (isShootReady) {
            kickerSys.kickIt();
            isShootReady = false;
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    shooterSys.stop();
                    kickerSys.kickReady();
                }
            }, 200);

            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    spindexerSys.advanceOneSlot();
                }
            }, 500);

            return true;
        }

        return false;
    }

    public void incrementSpindexerSlot() {
        spindexerSys.advanceOneSlot();
    }

    public void decrementSpindexerSlot() {
        spindexerSys.decreaseOneSlot();
    }

    public void setSpindexerSlot(int index) {
        spindexerSys.rotateToSlot(index);
    }

    public void increaseShooterRpmFar(){
        shooterSys.increaseRpmFar();
    }

    public void decreaseShooterRpmFar(){
        shooterSys.decreaseRpmFar();
    }

    public void increaseSpindexer() {
        spindexerSys.nudgePosition(5);
    }
    public void decreaseSpindexer() {
        spindexerSys.nudgePosition(-5);
    }

    public void setShooterTargetRange(boolean near){
        shooterSys.setTargetRange(near);
    }

    public void toggleLimelightPipeline() {
        limelightSys.togglePipeline();
    }

    public void resetIMU()  {
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
        }catch (InterruptedException ie){
            telemetryM.debug("IMU reset failed");
        }
    }

    private double getScaledTxOutput(double txDelta, double tolerance) {
        headingController.setTargetPosition(0);
        headingController.updateError(txDelta);
        double pidOutput = headingController.run(); // This now includes the F term

        return Math.abs(headingController.getError()) > tolerance
                ? MathFunctions.clamp(pidOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER)
                : 0;
    }

}
