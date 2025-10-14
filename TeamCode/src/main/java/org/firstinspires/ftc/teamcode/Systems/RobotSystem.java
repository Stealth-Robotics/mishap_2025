package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Timer;
import java.util.TimerTask;

public class RobotSystem {
    protected HardwareMap hardwareMap;
    protected SweeperSubSystem sweeperSys;
    protected HoodSubSystem hoodSys;
    protected KickerSubSystem kickerSys;
    protected ShooterSubSystem shooterSys;
    protected SpindexerSubSystem spindexerSys;

    private boolean isIntaking = false;

    private final Timer timer = new Timer();

    private boolean isShootReady = false;
    protected Follower follower;

    private Telemetry telemetry;

    public RobotSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.sweeperSys = new SweeperSubSystem(hardwareMap);
        this.hoodSys = new HoodSubSystem(hardwareMap);
        this.kickerSys = new KickerSubSystem(hardwareMap);
        this.shooterSys = new ShooterSubSystem(hardwareMap);
        this.spindexerSys = new SpindexerSubSystem(hardwareMap);
        this.follower = Constants.createFollower(hardwareMap);
        kickerSys.kickReady();
        this.telemetry = telemetry;
        //hoodSys.hoodIntake();
    }

    public void update() {
        follower.update();
        if (shooterSys.shootReady() && hoodSys.shootReady() && kickerSys.isKickReady()) {
            isShootReady = true;
        }

        // Robot Telemetry shown on screen
        telemetry.addData("Shooter RPM", shooterSys.getCurrentRPM());
        telemetry.update();
    }

    public Follower getFollower() {
        return follower;
    }

    public void startIntake()
    {
        isShootReady = false;
        isIntaking = true;
        kickerSys.kickReady();
        hoodSys.hoodIntake();
        sweeperSys.startIntake();
    }

    public void reverseIntake(){
        isShootReady = false;
        isIntaking = true;
        kickerSys.kickReady();
        hoodSys.hoodIntake();
        sweeperSys.reverseIntake();
    }

    public void stopIntake(){
        hoodSys.hoodShoot();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                sweeperSys.stopIntake();
                isIntaking = false;
            }
        }, 1000);
    }
    public void readyShoot()
    {
        if (!isIntaking) {
            kickerSys.kickReady();
            hoodSys.hoodShoot();
            shooterSys.shoot(.95);
        }
    }
    public void shoot() {
        if (isShootReady) {
            kickerSys.kickIt();
            isShootReady = false;
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    shooterSys.shoot(0);
                    kickerSys.kickReady();
                }
            }, 200);
        }
    }

    public void increaseShootPower()
    {
        shooterSys.increaseRPM();
    }

    public void decreaseShootPower()
    {
        shooterSys.decreaseRPM();
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

    public void increaseShooterRPM(){
        shooterSys.increaseRPM();
    }

    public void decreaseShooterRPM(){
        shooterSys.decreaseRPM();
    }

    public void resetIMU() throws InterruptedException {
        try {
            follower.getPoseTracker().getLocalizer().resetIMU();
        }catch (InterruptedException ie){
            // empty
        }

    }
}
