package Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class KickerSubSystem {
    Servo kickerServo;
    private static final double KICKER_UP = 0.5;
    private static final double KICKER_DOWN = 0.2;


    public KickerSubSystem(HardwareMap hardwareMap) {
        kickerServo = hardwareMap.get(Servo.class, "kicker_servo");
    }

    public void openKicker() {
        kickerServo.setPosition(KICKER_UP);
    }
    public void closeKicker() {
        kickerServo.setPosition(KICKER_DOWN);
    }
}
