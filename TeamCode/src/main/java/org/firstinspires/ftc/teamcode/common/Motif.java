package org.firstinspires.ftc.teamcode.common;

/**
 * Represents the determined motif (pattern) in the autonomous period.
 * Each motif is associated with a unique identifier.
 */
public enum Motif {
    /**
     * The motif is unknown or has not yet been determined.
     */
    UNKNOWN(-1),
    /**
     * Represents the Green-Purple-Purple motif.
     */
    GPP(21),
    /**
     * Represents the Purple-Green-Purple motif.
     */
    PGP(22),
    /**
     * Represents the Purple-Purple-Green motif.
     */
    PPG(23);

    /**
     * The unique integer identifier for the motif.
     */
    public final int id;

    /**
     * Constructs a Motif enum member with its specific ID.
     *
     * @param id The integer ID to associate with the motif.
     */
    Motif(int id) {
        this.id = id;
    }

    /**
     * Retrieves a Motif enum constant by its integer identifier.
     *
     * @param id The ID of the motif to find.
     * @return The corresponding Motif constant, or {@link #UNKNOWN} if no match is found.
     */
    public static Motif fromId(int id) {
        for (Motif m : values()) {
            if (m.id == id) {
                return m;
            }
        }
        return UNKNOWN;
    }
}
