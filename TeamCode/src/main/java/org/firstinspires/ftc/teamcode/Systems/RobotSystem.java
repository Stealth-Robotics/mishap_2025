package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.telemetry.JoinedTelemetry;
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

    private final Telemetry telemetry;

    public RobotSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.sweeperSys = new SweeperSubsystem(hardwareMap);
        this.hoodSys = new HoodSubsystem(hardwareMap);
        this.kickerSys = new KickerSubsystem(hardwareMap);
        this.shooterSys = new ShooterSubsystem(hardwareMap);
        this.spindexerSys = new SpindexerSubsystem(hardwareMap);
        this.follower = Constants.createFollower(hardwareMap);
        kickerSys.kickReady();
        this.telemetry = new JoinedTelemetry(telemetry);;
    }

    public void update() {
        follower.update();
        shooterSys.update();
        this.isShootReady = shooterSys.isShootReady()
                && hoodSys.shootReady()
                && kickerSys.isKickReady()
                && spindexerSys.isReady();

        // Robot Telemetry shown on screen
        telemetry.addData("Shooter RPM", shooterSys.getCurrentRpm());
        telemetry.addData("Target RPM", shooterSys.getTargetRpm());
        telemetry.addData("Spindexer ticks", spindexerSys.getPosition());
        telemetry.addData("ShootReady:", shooterSys.isShootReady());
        telemetry.addData("HoodReady:", hoodSys.shootReady());
        telemetry.addData("KickerReady:", kickerSys.isKickReady());
        telemetry.addData("SpindexerReady:", spindexerSys.isReady());
        telemetry.addData("IsIntaking:", isIntaking);
        telemetry.update();
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

    public void readyShoot()
    {
        if (!isIntaking && !isShootReady) {
            kickerSys.kickReady();
            hoodSys.hoodShoot();
            shooterSys.spinUpShooter();
        }
    }

    public void shoot() {
        if (isShootReady) {
            kickerSys.kickIt();
            isShootReady = false;
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    shooterSys.SpinDown();
                    kickerSys.kickReady();
                }
            }, 200);

            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    spindexerSys.advanceOneSlot();
                }
            }, 500);
        }
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
