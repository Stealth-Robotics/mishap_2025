package Systems;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class RobotSystem {
    protected HardwareMap hardwareMap;
    protected SweeperSubSystem sweeperSys;
    protected HoodSubSystem hoodSys;
    protected KickerSubSystem kickerSys;
    protected ShooterSubSystem shooterSys;
    protected SpindexerSubSystem spindexerSys;

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
}
