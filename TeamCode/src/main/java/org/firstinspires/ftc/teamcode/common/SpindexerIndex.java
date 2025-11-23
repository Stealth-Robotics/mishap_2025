package org.firstinspires.ftc.teamcode.common;

public class SpindexerIndex {
    public static int lastPosition = -1;
    private static boolean isValid = false;

    private static int shootSlot = -1;

    public static void setPosition(int pose, int slot) {
        lastPosition = pose;
        isValid = true;
        shootSlot = slot;
    }

    public static int getPosition() {
        return lastPosition;
    }

    public static int getShootSlot() {
        return shootSlot;
    }

    public static boolean getIsValid() {
        return isValid;
    }
    public static void setInvalid() {
        isValid = false;
    }
}
