package org.firstinspires.ftc.teamcode.systems;

import static org.firstinspires.ftc.teamcode.common.AprilTagIds.fromId;
import static org.firstinspires.ftc.teamcode.common.Pipeline.*;

import androidx.annotation.NonNull;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class LimelightSubsystem {

    // Name of the hardware device
    public static final String NAME = "limelight";

    // The length of time to keep LLResults in the queue before removing them.
    private static final long QUEUE_DEFAULT_TIMEOUT = 1000; // X milliseconds

    // Rotational difference between limelight and Pedro yaw
    private static final double LL_ANGLE_DELTA = 90;

    // How many degrees back is your limelight rotated from perfectly vertical?
    private  static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 20.1;

    // Distance from the center of the Limelight lens to the floor
    private static final double LIMELIGHT_FLOOR_HEIGHT_INCHES =9.0;

    // Used to calculate distance from an APRIL tag
    private static final double APRIL_TAG_CENTER_HEIGHT_INCHES = 27.5f; //6.125;

    // Used to calculate distance from a sample target laying flat on the floor
    private static final double ARTIFACT_TARGET_CENTER_HEIGHT_INCHES = 2.5f;
    private final Limelight3A limelight;
    private LLResult lastResult;
    // A queue of the last few LLResults in the last QUEUE_DEFAULT_TIMEOUT window
    private final Deque<LLResult> resultsQueue = new LinkedList<>();
    private final TelemetryManager telemetryA; // TODO: Rename

    /**
     * Initialize the limelight camera and default to pipeline 1 both
     * @param hardwareMap hardware map
     */
    public LimelightSubsystem(HardwareMap hardwareMap) {
        this.telemetryA = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, NAME);
        limelight.start();
        limelight.pipelineSwitch(APRILTAG_TARGET_BOTH.id);

    }

    /**
     * Called every loop to update camera information
     */
    public void update(){
        LLResult result = this.getLastResult();
        if (result != null && result.isValid() && result.getStaleness() < 50) {
            telemetryA.addData("CAM Latency", result.getStaleness());
            telemetryA.addData("AprilTag ID", this.getAprilTagId());
            // TODO: this should probably be moved out of the update loop
            Pose3D botPose = this.getAveragePose3D(100);
            if (botPose != null) {
                Pose convertedPose = getFollowPoseFromLimelight(botPose);
                telemetryA.addData("Bot Pose Fixed X:", convertedPose.getX());
                telemetryA.addData("Bot Pose Fixed Y:", convertedPose.getY());
                telemetryA.addData("Bot Heading Fixed: ", Math.toDegrees(convertedPose.getHeading()));
            }
        }
    }

    /**
     * Converts a Pose3D that Limelight uses to a Pose that the Follower uses
     */
    @NonNull
    public static Pose getFollowPoseFromLimelight(Pose3D botPose) {

        //TODO: this doesn't seem to work as expected rolling our own
//        return new Pose(
//                botPose.getPosition().x,
//                botPose.getPosition().y,
//                botPose.getOrientation().getYaw(),
//                FTCCoordinates.INSTANCE
//        ).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        // NOTE: The code below is the manual, more complex way of doing the conversion.
        // It is commented out because the single line above is preferred.
        // If you still want to use this manual code, you will need to resolve
        // where to get variables like 'LL_ANGLE_DELTA', 'Constants', etc.
        // Also, this method only receives 'botPose', but the code below needs 'result'.

        Pose followPose = new Pose(); // Initialize the pose

        // Camera turning CC from 0 goes to -180
        // Camera turning CW from 0 goes to 180
        double llYaw = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
        double imuAngle = (llYaw < 0) ? (360 + llYaw - LL_ANGLE_DELTA) : (llYaw - LL_ANGLE_DELTA);

        // Ensure within 0-360 range
        imuAngle %= 360;

        // Convert to pedro heading
        followPose = followPose.setHeading(Math.toRadians(imuAngle));

        // limelight starting field position is in the middle of the field
        // and is in meters.
        // pedro starting position is red alliance human corner
        // Pedro.X, Y is 0 to 144 inches
        // Limelight.X, Y is -1.8288 to +1.8288

        // first convert to inches
        Position position = botPose.getPosition().toUnit(DistanceUnit.INCH);

        // now translate to pedro coordinates for DECODE 2025 field
        // Note X and Y are swapped
        followPose = followPose.withY(-position.x + (Constants.FIELD_SIZE_X_INCHES / 2));
        followPose = followPose.withX((Constants.FIELD_SIZE_Y_INCHES / 2) - (-position.y));
        return followPose;    }

        /**
         * Gets the last target seen of the selected pipeline
         * @return last LLResult or null if none
         */
    public LLResult getLastResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            this.lastResult = result;
            this.addResultToQueue(result);
        }

        return lastResult;
    }

    /**
     * Gets direct access to the Limelight3A object
     * @return Limelight3A object
     */
    public Limelight3A getLimelight(){
        return limelight;
    }

    /**
     * Switches the Limelight pipeline
     * @param pipeline Enum of the pipeline to switch to
     */
    public void setPipeline(@NonNull Pipeline pipeline) {
        limelight.pipelineSwitch(pipeline.id);
        this.lastResult = null;
        resultsQueue.clear();
    }

    /**
     * Switches the pipeline to the next one in the enum
     */
    public void togglePipeline(){
        Pipeline cur = Pipeline.values()[limelight.getStatus().getPipelineIndex()];
        switch (cur){
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
                setPipeline(Pipeline.APRILTAG_TARGET_BLUE);
                break;
        }
    }

    /**
     * Calculates the average of the LLResults over QUEUE_DEFAULT_TIMEOUT in the queue
     * @return Pose3D of the average
     */
    public Pose3D getAveragePose3D() {
        return getAveragePose3D(QUEUE_DEFAULT_TIMEOUT);
    }

    /**
     * Calculates the average of the LLResults over the latencyMs in the queue
     * @return Pose3D of the average
     */
    public Pose3D getAveragePose3D(double latencyMs) {
        List<Pose3D> poses = new ArrayList<>();
        List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();
        List<Double> yawValues = new ArrayList<>(); // List to store yaw values

        // Extract Pose3D, calculate x, y, and yaw averages
        for (LLResult result : this.resultsQueue) {
            if (result.isValid() && result.getStaleness() < latencyMs) {
                Pose3D pose3D = result.getBotpose();
                if (pose3D != null) {
                    poses.add(pose3D);
                    xValues.add(pose3D.getPosition().x);
                    yValues.add(pose3D.getPosition().y);

                    // Get yaw and add to list
                    yawValues.add(pose3D.getOrientation().getYaw()); // Assuming firstAngle is yaw
                }
            }
        }

        if (poses.isEmpty()) {
            return null; // Handle empty case
        }

        // Calculate average and standard deviation for x, y, and yaw
        double xAvg = calculateAverage(xValues);
        double yAvg = calculateAverage(yValues);
        double yawAvg = calculateAverage(yawValues); // Average yaw
        double xStdDev = calculateStandardDeviation(xValues, xAvg);
        double yStdDev = calculateStandardDeviation(yValues, yAvg);
        double yawStdDev = calculateStandardDeviation(yawValues, yawAvg); // Standard deviation for yaw

        // Filter outliers, including yaw in the condition
        List<Pose3D> filteredPoses = new LinkedList<>();
        for (int i = 0; i < poses.size(); i++) {
            Pose3D pose = poses.get(i);
            if (Math.abs(pose.getPosition().x - xAvg) <= 2 * xStdDev &&
                    Math.abs(pose.getPosition().y - yAvg) <= 2 * yStdDev &&
                    Math.abs(yawValues.get(i) - yawAvg) <= 2 * yawStdDev) { // Check yaw outlier
                filteredPoses.add(pose);
            }
        }

        // Calculate average of filtered poses, including yaw
        double filteredXAvg = calculateAveragePose3D(filteredPoses, "x");
        double filteredYAvg = calculateAveragePose3D(filteredPoses, "y");
        double filteredYawAvg = calculateAverageYaw(filteredPoses); // Average filtered yaw

        // Use the last Pose3d as a template to create a new one with the average values
        Pose3D lastPose3d = resultsQueue.peekLast() != null ? resultsQueue.peekLast().getBotpose() : null;
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
     * Calculates the average of getTx() and getTy() for results within a certain latency.
     * This method filters out outliers that are more than 2 standard deviations from the mean.
     * @param latencyMs The maximum age of a result in milliseconds to be included in the average.
     * @return A double array containing the average tx [0] and ty [1], or null if no valid results are found.
     */
    public Pose getAverageTxTy(long latencyMs) {
        List<Double> txValues = new ArrayList<>();
        List<Double> tyValues = new ArrayList<>();

        // The queue is ordered from oldest to newest.
        for (LLResult result : getResultsQueue()) {
            if (result.isValid() && result.getStaleness() < latencyMs) {
                txValues.add(result.getTx());
                tyValues.add(result.getTy());
            }
        }

        if (txValues.isEmpty()) {
            return null;
        }

        // Calculate average and standard deviation for tx and ty
        double txAvg = calculateAverage(txValues);
        double tyAvg = calculateAverage(tyValues);
        double txStdDev = calculateStandardDeviation(txValues, txAvg);
        double tyStdDev = calculateStandardDeviation(tyValues, tyAvg);

        // Filter outliers
        List<Double> filteredTxValues = new ArrayList<>();
        List<Double> filteredTyValues = new ArrayList<>();
        for (int i = 0; i < txValues.size(); i++) {
            // Only include values within 2 standard deviations of the mean
            if (Math.abs(txValues.get(i) - txAvg) <= 2 * txStdDev &&
                    Math.abs(tyValues.get(i) - tyAvg) <= 2 * tyStdDev) {
                filteredTxValues.add(txValues.get(i));
                filteredTyValues.add(tyValues.get(i));
            }
        }

        // If all values were filtered out, just return the unfiltered average
        if (filteredTxValues.isEmpty()) {
            return new Pose(txAvg, tyAvg, 0);
        }

        // Calculate average of filtered values
        double filteredTxAvg = calculateAverage(filteredTxValues);
        double filteredTyAvg = calculateAverage(filteredTyValues);
        return  new Pose(filteredTxAvg, filteredTyAvg, 0);
    }

     /**
     * gets the ID of the first AprilTag in the Fiducial list
     * @return AprilTagId or TAG_ID_NONE if not found or not selected
     */
    public AprilTagIds getAprilTagId()
    {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()){
            return AprilTagIds.TAG_ID_NONE;
        }

        Pipeline cur = Pipeline.values()[result.getPipelineIndex()];
        return switch (cur) {
            case APRILTAG_TARGET_BLUE, APRILTAG_TARGET_RED, APRILTAG_MOTIF, APRILTAG_TARGET_BOTH -> {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    // Find the matching enum by the ID
                    yield  AprilTagIds.fromId(fiducials.get(0).getFiducialId());
                }

                yield  AprilTagIds.TAG_ID_NONE;
            }
            default -> AprilTagIds.TAG_ID_NONE;
        };
    }

    /**
     * Returns the LLResult queue. for doing averaging or prediction
     * @return Deque of LLResults
     */
    private Deque<LLResult> getResultsQueue() {
        return resultsQueue;
    }

    /**
     * Adds a LLResult to the queue
     * @param result LLResult to add
     */
    private void addResultToQueue(LLResult result) {
        // Remove elements older than QUEU_DEFAULT_TIMEOUT
        while (!resultsQueue.isEmpty()
                && (resultsQueue.peekFirst() != null)
                && (resultsQueue.peekFirst().getStaleness() > QUEUE_DEFAULT_TIMEOUT)) {
            resultsQueue.removeFirst();
        }

        resultsQueue.addLast(result);
    }

    /**
     * Calculates the average of a list of values
     * @param values list of values
     * @return average
     */
    private static double calculateAverage(List<Double> values) {
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
     * Calculates the standard deviation of a list of values
     * @param values list of values
     * @param average average of the values
     * @return standard deviation
     */
    private static double calculateStandardDeviation(List<Double> values, double average) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sumOfSquaredDeviations = 0.0;
        for (Double value : values) {
            sumOfSquaredDeviations += Math.pow(value - average, 2);
        }
        return Math.sqrt(sumOfSquaredDeviations / (values.size() - 1));
    }

    /**
     * Calculates the average of a list of Pose3D objects
     * @param poses list of Pose3D objects
     * @param coordinate x or y
     * @return average
     */
    private static double calculateAveragePose3D(List<Pose3D> poses, String coordinate) {
        if (poses.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Pose3D pose : poses) {
            if (coordinate.equals("x")) {
                sum += pose.getPosition().x;
            } else if (coordinate.equals("y")) {
                sum += pose.getPosition().y;
            }
        }
        return sum / poses.size();
    }

    /**
     * Calculates the average yaw of a list of Pose3D objects
     * @param poses list of Pose3D objects
     * @return average yaw
     */
    private static double calculateAverageYaw(List<Pose3D> poses) {
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
