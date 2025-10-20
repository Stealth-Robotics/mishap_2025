package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;
import java.util.TimerTask;

public class RobotSystem {
    protected HardwareMap hardwareMap;
    protected SweeperSubsystem sweeperSys;
    protected HoodSubsystem hoodSys;
    protected KickerSubsystem kickerSys;
    protected ShooterSubsystem shooterSys;
    protected SpindexerSubsystem spindexerSys;

    public static boolean isIntaking = false;

    private final Timer timer = new Timer();

    private volatile boolean isShootReady = false;
    protected Follower follower;

    private final TelemetryManager telemetryM;
    private final Telemetry telemetry;

    public RobotSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.sweeperSys = new SweeperSubsystem(hardwareMap);
        this.hoodSys = new HoodSubsystem(hardwareMap);
        this.kickerSys = new KickerSubsystem(hardwareMap);
        this.shooterSys = new ShooterSubsystem(hardwareMap);
        this.spindexerSys = new SpindexerSubsystem(hardwareMap);
        this.follower = Constants.createFollower(hardwareMap);
        this.telemetry = telemetry;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void update() {
        follower.update();
        shooterSys.update();
        this.isShootReady = shooterSys.isReadyToShoot()
                && hoodSys.shootReady()
                && kickerSys.isKickReady()
                && spindexerSys.isReady();

        // Robot Telemetry shown on screen
        telemetryM.addData("Shooter RPM", shooterSys.getCurrentRpm());
        telemetryM.addData("Target RPM", shooterSys.getTargetRpm());
        telemetryM.addData("Spindexer ticks", spindexerSys.getPosition());
        telemetryM.addData("ShootReady:", shooterSys.isReadyToShoot());
        telemetryM.addData("HoodReady:", hoodSys.shootReady());
        telemetryM.addData("KickerReady:", kickerSys.isKickReady());
        telemetryM.addData("SpindexerReady:", spindexerSys.isReady());
        telemetryM.addData("IsIntaking:", isIntaking);
        telemetryM.update(telemetry);
    }

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
        hoodSys.hoodShoot();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                sweeperSys.stopIntake();
                spindexerSys.setBrake();
                // should check for ball in slot
                spindexerSys.advanceOneSlot();
                isIntaking = false;
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

    public void resetIMU() throws InterruptedException {
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
        }catch (InterruptedException ie){
            // empty
        }
    }
}
