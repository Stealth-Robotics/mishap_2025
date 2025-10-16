package org.firstinspires.ftc.teamcode.Systems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    private double curShootPower = ShooterSubSystem.SHOOT_DEFAULT_POWER;

    private boolean isIntaking = false;

    private Timer timer = new Timer();

    private boolean isShootReady = false;
    protected Follower follower;

    public RobotSystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.sweeperSys = new SweeperSubSystem(hardwareMap);
        this.hoodSys = new HoodSubSystem(hardwareMap);
        this.kickerSys = new KickerSubSystem(hardwareMap);
        this.shooterSys = new ShooterSubSystem(hardwareMap);
        this.spindexerSys = new SpindexerSubSystem(hardwareMap);
        this.follower = Constants.createFollower(hardwareMap);

    }

    public void update() {
        follower.update();
    }

    public void startIntake(){
        isIntaking = true;
        hoodSys.hoodIntake();
        sweeperSys.startIntake();
    }

    public void initTelOp(){
        kickerSys.slapItBack();
        hoodSys.hoodShoot();
    }

    public void toggleIntake(){
        if(isIntaking){
            stopIntake();
        }else{
            startIntake();
        }
    }
    public void stopIntake(){
        hoodSys.hoodShoot();
        timer.schedule(
                new TimerTask() {
                    @Override
                    public void run() {
                        sweeperSys.stopIntake();
                        isIntaking = false;
                    }
                },
                500
        );
    }

    public Follower getFollower() {
        return follower;
    }

    public void resetImu() throws InterruptedException {
        try{
            follower.getPoseTracker().getLocalizer().resetIMU();

        }catch(InterruptedException e){
            //EMPTY
        }

    }

}
