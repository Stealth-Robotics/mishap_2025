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
        kickerSys.kickReady();
        hoodSys.hoodIntake();
    }

    public void update() {
        follower.update();
        if (shooterSys.shootReady() && hoodSys.shootReady() && kickerSys.isKickReady()) {
            isShootReady = true;
        }

//        if(hoodSys.shootReady() && sweeperSys.isSweeperOn())
//        {
//            //sweeperSys.stopIntake();
//        }


    }

    public Follower getFollower() {
        return follower;
    }

    public void startIntake()
    {
        isShootReady = false;
        isIntaking = true;
        sweeperSys.startIntake();
        kickerSys.kickReady();
        hoodSys.hoodIntake();

    }

    public void stopIntake(){
        hoodSys.hoodShoot();
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                sweeperSys.stopIntake();
                isIntaking = false;
            }
        }, 500);
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
            new Timer().schedule(new TimerTask() {
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
        if(curShootPower < ShooterSubSystem.SHOOT_MAX_POWER)
        {
            curShootPower += .02;
            shooterSys.shoot(curShootPower);
        }
    }

    public void decreaseShootPower()
    {
        if(curShootPower > ShooterSubSystem.SHOOT_DEFAULT_POWER)
        {
            curShootPower -= .02;
            shooterSys.shoot(curShootPower);
        }
    }

    public void setSpindexerPower(double power) {
        spindexerSys.spin(power);
    }
}
