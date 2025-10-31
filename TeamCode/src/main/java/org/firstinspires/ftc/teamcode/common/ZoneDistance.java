package org.firstinspires.ftc.teamcode.common;

// setup various feild zone for shooter power and paddle angle
public enum ZoneDistance {
    FAR(80.0) ,
    MID(55.0),
    NEAR(0.0);

    public final double id;

    ZoneDistance(double id) {
        this.id = id;
    }

    public ZoneDistance fromId(double id) {
        for (ZoneDistance z : ZoneDistance.values()) {
            if (z.id == id) {
                return z;
            }
        }

        return ZoneDistance.FAR;
    }
}