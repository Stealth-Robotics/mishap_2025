package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color; // <-- IMPORT THIS FOR HSV CONVERSION

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.SlotState;

@Configurable
public class ColorSensorSubsystem {
    private final RevColorSensorV3 colorSensor;

    private SlotState lastDetection = SlotState.UNKNOWN;
    private final TelemetryManager telemetryM;

    // --- HSV COLOR PROFILES ---
    // These values need to be tuned experimentally.
    // Format: {H_min, H_max, S_min, S_max, V_min, V_max}
    // Hue is 0-360, Saturation and Value are 0.0-1.0

    // --- CLOSE RANGE (e.g., 20-58mm) ---
    public static double[] greenArtifactCloseHSV = {150, 168, 0.6, .74, 0.08, .22};
    public static double[] purpleArtifactCloseHSV = {170, 240, 0.4, .7, .04, .25};

    // --- FAR RANGE (e.g., 58-72mm) ---
    public static double[] greenArtifactFarHSV = {150, 167, 0.6, .8, 0.07, .091};
    public static double[] purpleArtifactFarHSV = {169, 240, .4, .59, 0.05, .2};

    // --- DISTANCE THRESHOLDS ---
    private static final double MIN_DISTANCE_MM = 20;
    private static final double MAX_DISTANCE_MM = 72;
    // This value separates "close" from "far" range. Tune this as needed.
    private static final double CLOSE_FAR_THRESHOLD_MM = 58;

    // --- High/Low Telemetry Variables ---
    private double
            hueMax = -1, hueMin = 1000,
            satMax = -1, satMin = 400,
            valMax = -1, valMin = 400,
            distanceMin = 5000, distanceMax = -1;

    public ColorSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensor.setGain(100); // High gain is good for detecting colors
    }

    public void resetHighLow() {
        hueMax = -1; hueMin = 4000;
        satMax = -1; satMin = 200;
        valMax = -1; valMin = 200;
        distanceMin = 5000; distanceMax = -1;
    }

    public void update() {
        lastDetection = getDetectedColor();
//        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetryM.addData("Detected Color", lastDetection);
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (colors.red * 255), (int) (colors.green * 255), (int) (colors.blue * 255), hsv);
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        // Update high/low values
        if (hsv[0] > hueMax) hueMax = hsv[0];
        if (hsv[0] < hueMin) hueMin = hsv[0];
        if (hsv[1] > satMax) satMax = hsv[1];
        if (hsv[1] < satMin) satMin = hsv[1];
        if (hsv[2] > valMax) valMax = hsv[2];
        if (hsv[2] < valMin) valMin = hsv[2];
        if (distance > distanceMax) distanceMax = distance;
        if (distance < distanceMin) distanceMin = distance;

        telemetryM.addData("Distance (mm)", distance);
        telemetryM.addData("Range", distance < CLOSE_FAR_THRESHOLD_MM ? "Close" : "Far");
        telemetryM.addData("Hue", hsv[0]);
        telemetryM.addData("Saturation", hsv[1]);
        telemetryM.addData("Value", hsv[2]);

        telemetryM.addData("--- High/Low Tracking ---", ""); // Empty value for a header line
        telemetryM.addData("Hue Min / Max", hueMin + " / " + hueMax);
        telemetryM.addData("Sat Min / Max", satMin + " / " + satMax);
        telemetryM.addData("Val Min / Max", valMin + " / " + valMax);
        telemetryM.addData("Dist Min / Max", distanceMin + " / " + distanceMax);
    }

    private SlotState getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        if (distance < MIN_DISTANCE_MM || distance > MAX_DISTANCE_MM) {
            return SlotState.EMPTY;
        }

        // Convert sensor's RGB readings to HSV
        float[] hsv = new float[3];
        Color.RGBToHSV((int) (colors.red * 255), (int) (colors.green * 255), (int) (colors.blue * 255), hsv);

        // --- Detection Logic based on Distance ---
        if (distance < CLOSE_FAR_THRESHOLD_MM) {
            // CLOSE RANGE DETECTION
            if (isMatchHSV(hsv, greenArtifactCloseHSV)) return SlotState.ARTIFACT_GREEN;
            if (isMatchHSV(hsv, purpleArtifactCloseHSV)) return SlotState.ARTIFACT_PURPLE;
        } else {
            // FAR RANGE DETECTION
            if (isMatchHSV(hsv, greenArtifactFarHSV)) return SlotState.ARTIFACT_GREEN;
            if (isMatchHSV(hsv, purpleArtifactFarHSV)) return SlotState.ARTIFACT_PURPLE;
        }

        return SlotState.UNKNOWN;
    }

    /**
     * Checks if the given HSV values fall within the target HSV range.
     * @param hsvValues The current {H, S, V} from the sensor.
     * @param targetHsv The target range {H_min, H_max, S_min, S_max, V_min, V_max}.
     * @return True if the color is a match.
     */
    private boolean isMatchHSV(float[] hsvValues, double[] targetHsv) {
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        boolean hueMatch = (hue >= targetHsv[0] && hue <= targetHsv[1]);
        boolean satMatch = (saturation >= targetHsv[2] && saturation <= targetHsv[3]);
        boolean valMatch = (value >= targetHsv[4] && value <= targetHsv[5]);

        // only matching Hue for now
        return hueMatch; //&& satMatch && valMatch;
    }

    /**
     * Original RGB matching logic, kept for achromatic objects like the white paddle.
     */
    private boolean isMatchRGB(
            NormalizedRGBA colors,
            double redHigh, double redLow,
            double greenHigh, double greenLow,
            double blueHigh, double blueLow) {
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        return (r <= redHigh && r >= redLow)
                && (g <= greenHigh && g >= greenLow)
                && (b <= blueHigh && b >= blueLow);
    }

    public SlotState getLastDetection() {
        return lastDetection;
    }
}
