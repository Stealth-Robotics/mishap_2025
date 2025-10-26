package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.common.Pipeline.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.AprilTagIds;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;

/**
 * Manages all interactions with the Limelight camera, including pipeline switching,
 * data retrieval, and pose calculations.
 */
public class LimelightSubsystem {

    /** The name of the hardware device in the robot's configuration. */
    public static final String NAME = "limelight";

    /** The length of time to keep LLResults in the queue before removing them, in milliseconds. */
    private static final long QUEUE_DEFAULT_TIMEOUT_MS = 1000;

    /** Rotational difference in degrees between the Limelight's yaw and the robot's IMU yaw. */
    private static final double LL_ANGLE_DELTA = 90;

    /** How many degrees back the Limelight is rotated from perfectly vertical. */
    private static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 20.1;

    /** Distance from the center of the Limelight lens to the floor, in inches. */
    private static final double LIMELIGHT_FLOOR_HEIGHT_INCHES = 9.1;

    /** The height of the center of an AprilTag from the floor, in inches. */
    private static final double APRIL_TAG_CENTER_HEIGHT_INCHES = 27.5; // Changed from 'f' suffix to double

    /** The height of the center of an artifact target from the floor, in inches. */
    private static final double ARTIFACT_TARGET_CENTER_HEIGHT_INCHES = 2.5;

    /** The Limelight camera instance. */
    private final Limelight3A limelight;

    /** The last valid result received from the Limelight. */
    private LLResult lastResult;

    /** A queue of recent LLResults for averaging and filtering. */
    private final Deque<LLResult> resultsQueue = new LinkedList<>();

    /** The telemetry manager for displaying debug information. */
    private final TelemetryManager telemetryM;

    /**
     * Initializes the Limelight subsystem.
     *
     * @param hardwareMap The robot's hardware map.
     */
    public LimelightSubsystem(@NonNull HardwareMap hardwareMap) {
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, NAME);
        limelight.start();
        limelight.pipelineSwitch(APRILTAG_TARGET_BOTH.id);
    }

    /**
     * This method should be called in every iteration of the main robot loop to
     * update telemetry with the latest camera data.
     */
    public void update() {
        LLResult result = this.getLastResult();
        telemetryM.addData("Pipeline", this.getCurrentPipeline());
        if (result != null && result.isValid() && result.getStaleness() < 100) {
            telemetryM.addData("AprilTag ID", this.getAprilTagId());
            Pose targetPose = this.getAvgTargetPose(50);
            if (targetPose != null) {
                telemetryM.addData("Target Distance Ty: ", calcGoalDistanceByTy(targetPose.getY()));
            }


/*  NOTE Uncomment the below to show bot position on field
            Pose3D botPose = this.getAvgBotPose(20);
            if (botPose != null) {
                Pose convertedPose = limelightToPedroPose(botPose);
                telemetryM.addData("Bot Pose Fixed X:", convertedPose.getX());
                telemetryM.addData("Bot Pose Fixed Y:", convertedPose.getY());
                telemetryM.addData("Bot Heading Fixed: ", Math.toDegrees(convertedPose.getHeading()));
            }
 */
        }
    }

    /**
     * Converts a Limelight-provided {@link Pose3D} into a {@link Pose} object
     * compatible with the path-following coordinate system.
     *
     * @param botPose The {@link Pose3D} from the Limelight.
     * @return The converted {@link Pose}.
     */
    @NonNull
    public static Pose limelightToPedroPose(@NonNull Pose3D botPose) {

        /* TODO: Figure out why the Pedro built in converter isn't working
        Position pose = botPose.getPosition().toUnit(DistanceUnit.INCH);
        return new Pose(
                    pose.x, // not sure but may need to swap x y
                    pose.y,
                    botPose.getOrientation().getYaw(AngleUnit.RADIANS),
                    FTCCoordinates.INSTANCE
            ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);

     */
        double llYaw = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
        // Rotoate the limlight by LL_ANGLE_DELTA to get the robot's heading
        double imuAngle = (llYaw < 0) ? (360 + llYaw - LL_ANGLE_DELTA) : (llYaw - LL_ANGLE_DELTA);

        // Ensure heading is within the 0-360 degree range
        imuAngle %= 360;

        // Convert position from Limelight's coordinate system (meters, field center)
        // to the path follower's system (inches, red alliance corner).
        Position position = botPose.getPosition().toUnit(DistanceUnit.INCH);

        // Note: X and Y are swapped and adjusted for the different origins.
        return new Pose(
                -position.x + (Constants.FIELD_SIZE_X_INCHES / 2.0),
                (Constants.FIELD_SIZE_Y_INCHES / 2.0) - (-position.y),
                Math.toRadians(imuAngle));
    }

    /**
     * Retrieves the most recent {@link LLResult} from the Limelight, updating the internal cache.
     *
     * @return The latest result, or the last known valid result if the new one is invalid.
     */
    @Nullable
    public LLResult getLastResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            this.lastResult = result;
            this.addResultToQueue(result);
        }
        return lastResult;
    }

    /**
     * Switches the Limelight's active pipeline.
     *
     * @param pipeline The {@link Pipeline} to switch to.
     * @return {@code true} if the pipeline was switched, {@code false} if it was already active.
     */
    public boolean setPipeline(@NonNull Pipeline pipeline) {
        if (pipeline == getCurrentPipeline()) {
            return false;
        }

        limelight.pipelineSwitch(pipeline.id);
        this.lastResult = null;
        resultsQueue.clear();
        return true;
    }

    /**
     * Gets the currently active pipeline on the Limelight.
     *
     * @return The current {@link Pipeline}.
     */
    public Pipeline getCurrentPipeline() {
        return Pipeline.values()[limelight.getStatus().getPipelineIndex()];
    }

    public void toggleTargetPileline() {
        Pipeline cur = getCurrentPipeline();
        switch (cur) {
            case APRILTAG_TARGET_BLUE:
                setPipeline(APRILTAG_TARGET_RED);
                break;
            case APRILTAG_TARGET_RED:
            default:
                setPipeline(APRILTAG_TARGET_BLUE);
                break;
        }
    }

    /**
     * Switches the pipeline to the next one in a predefined sequence.
     */
    public void toggleFullPipeline() {
        Pipeline cur = getCurrentPipeline();
        switch (cur) {
            case APRILTAG_TARGET_BLUE:
                setPipeline(APRILTAG_TARGET_BOTH);
                break;
            case APRILTAG_TARGET_BOTH:
                setPipeline(Pipeline.APRILTAG_TARGET_RED);
                break;
            case APRILTAG_TARGET_RED:
                setPipeline(Pipeline.APRILTAG_MOTIF);
                break;
            case APRILTAG_MOTIF:
            default:
                setPipeline(Pipeline.APRILTAG_TARGET_BLUE);
                break;
        }
    }

    /**
     * Calculates the average robot pose based on recent Limelight results, filtered for outliers.
     * This method uses the default queue timeout.
     *
     * @return The average {@link Pose3D}, or {@code null} if no valid data is available.
     */
    @Nullable
    public Pose3D getAvgBotPose() {
        return getAvgBotPose(QUEUE_DEFAULT_TIMEOUT_MS);
    }

    /**
     * Calculates the average robot pose based on Limelight results within a specified time window,
     * filtering for outliers.
     *
     * @param latencyMs The maximum age of a result in milliseconds to be included.
     * @return The average {@link Pose3D}, or {@code null} if no valid data is available.
     */
    @Nullable
    public Pose3D getAvgBotPose(double latencyMs) {
        List<Pose3D> poses = new ArrayList<>();
        List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();
        List<Double> yawValues = new ArrayList<>();

        for (LLResult result : this.resultsQueue) {
            if (result.isValid() && result.getStaleness() < latencyMs) {
                Pose3D pose3D = result.getBotpose();
                if (pose3D != null) {
                    poses.add(pose3D);
                    xValues.add(pose3D.getPosition().x);
                    yValues.add(pose3D.getPosition().y);
                    yawValues.add(pose3D.getOrientation().getYaw());
                }
            }
        }

        if (poses.isEmpty()) {
            return null;
        }

        double xAvg = calculateAverage(xValues);
        double yAvg = calculateAverage(yValues);
        double yawAvg = calculateAverage(yawValues);
        double xStdDev = calculateStandardDeviation(xValues, xAvg);
        double yStdDev = calculateStandardDeviation(yValues, yAvg);
        double yawStdDev = calculateStandardDeviation(yawValues, yawAvg);

        List<Pose3D> filteredPoses = new LinkedList<>();
        for (int i = 0; i < poses.size(); i++) {
            Pose3D pose = poses.get(i);
            if (Math.abs(pose.getPosition().x - xAvg) <= 2 * xStdDev &&
                    Math.abs(pose.getPosition().y - yAvg) <= 2 * yStdDev &&
                    Math.abs(yawValues.get(i) - yawAvg) <= 2 * yawStdDev) {
                filteredPoses.add(pose);
            }
        }

        double filteredXAvg = calculateAveragePose3D(filteredPoses, "x");
        double filteredYAvg = calculateAveragePose3D(filteredPoses, "y");
        double filteredYawAvg = calculateAverageYaw(filteredPoses);

        Pose3D lastPose3d = resultsQueue.peekLast() != null ? Objects.requireNonNull(resultsQueue.peekLast()).getBotpose() : null;
        if (lastPose3d == null) {
            return null;
        }

        return new Pose3D(
                new Position(
                        lastPose3d.getPosition().unit,
                        filteredXAvg,
                        filteredYAvg,
                        lastPose3d.getPosition().z,
                        lastPose3d.getPosition().acquisitionTime),
                new YawPitchRollAngles(
                        AngleUnit.DEGREES,
                        filteredYawAvg,
                        lastPose3d.getOrientation().getPitch(),
                        lastPose3d.getOrientation().getRoll(),
                        lastPose3d.getOrientation().getAcquisitionTime()));
    }

    /**
     * Calculates the average average target pose filtered for outliers.
     * NOTE: the heading is overloaded to return the target Avagerage Ta not heading
     * @param latencyMs The maximum age of a result in milliseconds to be included.
     * @return A {@link Pose} containing the average tx in X and ty in Y, or {@code null} if no valid data is available.
     */
    @Nullable
    public Pose getAvgTargetPose(long latencyMs) {
        List<Double> txValues = new ArrayList<>();
        List<Double> tyValues = new ArrayList<>();
        List<Double> taValues = new ArrayList<>();

        for (LLResult result : getResultsQueue()) {
            if (result.isValid() && result.getStaleness() < latencyMs) {
                taValues.add(result.getTa());
                txValues.add(result.getTx());
                tyValues.add(result.getTy());
            }
        }

        if (txValues.isEmpty()) {
            return null;
        }

        double txAvg = calculateAverage(txValues);
        double tyAvg = calculateAverage(tyValues);
        double taAvg = calculateAverage(taValues);
        double taStdDev = calculateStandardDeviation(taValues, taAvg);
        double txStdDev = calculateStandardDeviation(txValues, txAvg);
        double tyStdDev = calculateStandardDeviation(tyValues, tyAvg);

        List<Double> filteredTxValues = new ArrayList<>();
        List<Double> filteredTyValues = new ArrayList<>();
        List<Double> filteredTaValues = new ArrayList<>();

        for (int i = 0; i < txValues.size(); i++) {
            if (Math.abs(txValues.get(i) - txAvg) <= 2 * txStdDev
                    && Math.abs(tyValues.get(i) - tyAvg) <= 2 * tyStdDev
                    && Math.abs(taValues.get(i) - taAvg) <= 2 * taStdDev) {
                filteredTxValues.add(txValues.get(i));
                filteredTyValues.add(tyValues.get(i));
                filteredTaValues.add(taValues.get(i));
            }
        }

        if (filteredTxValues.isEmpty()) {
            return new Pose(txAvg, tyAvg, taAvg);
        }

        double filteredTxAvg = calculateAverage(filteredTxValues);
        double filteredTyAvg = calculateAverage(filteredTyValues);
        double filteredTaAvg = calculateAverage(filteredTaValues);

        return new Pose(filteredTxAvg, filteredTyAvg, filteredTaAvg);
    }

    /**
     * Gets the ID of the first visible AprilTag.
     *
     * @return The {@link AprilTagIds} enum corresponding to the detected tag, or {@code TAG_ID_NONE} if none is found.
     */
    public AprilTagIds getAprilTagId() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return AprilTagIds.TAG_ID_NONE;
        }

        Pipeline cur = Pipeline.values()[result.getPipelineIndex()];
        switch (cur) {
            case APRILTAG_TARGET_BLUE:
            case APRILTAG_TARGET_RED:
            case APRILTAG_MOTIF:
            case APRILTAG_TARGET_BOTH:
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    return AprilTagIds.fromId(fiducials.get(0).getFiducialId());
                }
                return AprilTagIds.TAG_ID_NONE;
            default:
                return AprilTagIds.TAG_ID_NONE;
        }
    }

    /**
     * Returns the queue of recent {@link LLResult} objects for external processing.
     *
     * @return A {@link Deque} of {@link LLResult}.
     */
    private Deque<LLResult> getResultsQueue() {
        return resultsQueue;
    }

    /**
     * Adds a result to the queue and removes old entries that have expired.
     *
     * @param result The {@link LLResult} to add.
     */
    private void addResultToQueue(@NonNull LLResult result) {
        while (!resultsQueue.isEmpty()
                && (resultsQueue.peekFirst() != null)
                && (resultsQueue.peekFirst().getStaleness() > QUEUE_DEFAULT_TIMEOUT_MS)) {
            resultsQueue.removeFirst();
        }
        resultsQueue.addLast(result);
    }

    /**
     * Calculates distance to a goal by using the cameras Ty value
     * @param ty The Ty value from the camera
     *
     * @return The distance to the goal in inches
     */
    public static double calcGoalDistanceByTy(double ty) {
        // This is the total angle from the camera to the target, relative to a horizontal line from the camera.
        double totalAngleDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + ty;

        // Convert the total angle to radians for use in Math.tan()
        double totalAngleRadians = Math.toRadians(totalAngleDegrees);

        // Calculate the height difference between the AprilTag and the Limelight
        double heightDifference = APRIL_TAG_CENTER_HEIGHT_INCHES - LIMELIGHT_FLOOR_HEIGHT_INCHES;

        // Using trigonometry (tangent) to find the horizontal distance
        // distance = opposite / tan(angle)

        return heightDifference / Math.tan(totalAngleRadians);    }

    /**
     * Calculates the arithmetic mean of a list of numbers.
     *
     * @param values The list of {@link Double} values.
     * @return The average.
     */
    private static double calculateAverage(@NonNull List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    /**
     * Calculates the sample standard deviation of a list of numbers.
     *
     * @param values  The list of {@link Double} values.
     * @param average The pre-calculated average of the values.
     * @return The standard deviation. Returns 0.0 if the list has fewer than two elements.
     */
    private static double calculateStandardDeviation(@NonNull List<Double> values, double average) {
        if (values.size() < 2) {
            return 0.0;
        }
        double sumOfSquaredDeviations = 0.0;
        for (Double value : values) {
            sumOfSquaredDeviations += Math.pow(value - average, 2);
        }
        return Math.sqrt(sumOfSquaredDeviations / (values.size() - 1.0));
    }

    /**
     * Calculates the average of a specific coordinate (X or Y) from a list of {@link Pose3D} objects.
     *
     * @param poses      The list of {@link Pose3D} objects.
     * @param coordinate A string, either "x" or "y", specifying the coordinate to average.
     * @return The average value for the specified coordinate.
     */
    private static double calculateAveragePose3D(@NonNull List<Pose3D> poses, @NonNull String coordinate) {
        if (poses.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Pose3D pose : poses) {
            if ("x".equalsIgnoreCase(coordinate)) {
                sum += pose.getPosition().x;
            } else if ("y".equalsIgnoreCase(coordinate)) {
                sum += pose.getPosition().y;
            }
        }
        return sum / poses.size();
    }

    /**
     * Calculates the average yaw from a list of {@link Pose3D} objects.
     *
     * @param poses The list of {@link Pose3D} objects.
     * @return The average yaw in degrees.
     */
    private static double calculateAverageYaw(@NonNull List<Pose3D> poses) {
        if (poses.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Pose3D pose : poses) {
            sum += pose.getOrientation().getYaw();
        }
        return sum / poses.size();
    }
}
