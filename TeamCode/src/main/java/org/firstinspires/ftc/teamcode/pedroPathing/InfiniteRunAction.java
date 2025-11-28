package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.paths.callbacks.PathCallback;

public class InfiniteRunAction implements PathCallback {
    private boolean initialized = false;

    private final PathCallback callback;

    public InfiniteRunAction(PathCallback callback) {
        this.callback = callback;
    }

    /**
     * This runs the callback.
     *
     * @return true if the action was successful
     */
    @Override
    public boolean run() {
        callback.run();
        return true;
    }

    /**
     * This checks if the callback is ready to run.
     *
     * @return true if the callback is ready to run
     */
    @Override
    public boolean isReady() {
        return callback.isReady() && initialized;
    }

    /**
     * This initializes the callback.
     */
    @Override
    public void initialize() {
        callback.initialize();
        initialized = true;
    }

    /**
     * This resets the callback.
     */
    @Override
    public void reset() {
        PathCallback.super.reset();
    }

    /**
     * This checks if the callback is completed.
     *
     * @return true if the callback is completed
     */
    @Override
    public boolean isCompleted() {
        return false;
    }

    /**
     * This returns the index of the path that this callback is for.
     *
     * @return the index of the path that this callback is for
     */
    @Override
    public int getPathIndex() {
        return callback.getPathIndex();
    }
}
