package org.firstinspires.ftc.teamcode.common;

/**
 * A simple debounce filter for boolean streams. Requires that the boolean change value from
 * baseline for a specified period of time before the filtered value changes.
 */
public class Debouncer {
    public enum DebounceType {
        kRising,
        kFalling,
        kBoth
    }

    private final double debounceTimeSeconds;
    private final DebounceType debounceType;
    private boolean baseline;

    private double prevTimeSeconds;

    /**
     * Creates a new Debouncer.
     *
     * @param debounceTime The number of seconds the value must change from baseline for the filtered
     *                     value to change.
     * @param type         Which type of state change the debouncing will be performed on.
     */
    public Debouncer(double debounceTime, DebounceType type) {
        debounceTimeSeconds = debounceTime;
        debounceType = type;

        resetTimer();

        switch (debounceType) {
            case kBoth: // fall-through
            case kRising:
                baseline = false;
                break;
            case kFalling:
                baseline = true;
                break;
            default:
                throw new IllegalArgumentException("Invalid debounce type!");
        }
    }

    /**
     * Creates a new Debouncer. Baseline value defaulted to "false."
     *
     * @param debounceTime The number of seconds the value must change from baseline for the filtered
     *                     value to change.
     */
    public Debouncer(double debounceTime) {
        this(debounceTime, DebounceType.kRising);
    }

        private void resetTimer() {
        // nanoTime gives us nanoseconds (1 billionth of a second), but we prefer to work in seconds
        // here. We can convert by dividing by 1000000000 which we can more easily express in
        // scientific notation as 1e9. We can also multiply by the inverse, 1e-9.
        prevTimeSeconds = System.nanoTime() * 1e-9;
    }

    private boolean hasElapsed() {
        return (System.nanoTime() * 1e-9) - prevTimeSeconds >= debounceTimeSeconds;
    }

    /**
     * Applies the debouncer to the input stream.
     *
     * @param input The current value of the input stream.
     * @return The debounced value of the input stream.
     */
    public boolean calculate(boolean input) {
        if (input == baseline) {
            resetTimer();
        }

        if (hasElapsed()) {
            if (debounceType == DebounceType.kBoth) {
                baseline = input;
                resetTimer();
            }
            return input;
        } else {
            return baseline;
        }
    }
}
