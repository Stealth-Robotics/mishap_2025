package org.firstinspires.ftc.teamcode.common;

public enum AprilTagIds {
    TAG_ID_NONE(-1),
    TAG_ID_TARGET_BLUE (20),
    TAG_ID_MOTIF_GPP(21),
    TAG_ID_MOTIF_PGP(22),
    TAG_ID_MOTIF_PPG(23),
    TAG_ID_TARGET_RED(24);
    public final int id;

    AprilTagIds(int id) {
        this.id = id;
    }

    public static AprilTagIds fromId(int id) {
        for (AprilTagIds tagId : AprilTagIds.values()) {
            if (tagId.id == id) {
                return tagId;
            }
        }

        return AprilTagIds.TAG_ID_NONE;
    }
}
