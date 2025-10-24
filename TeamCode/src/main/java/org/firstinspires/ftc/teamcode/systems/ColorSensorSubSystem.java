package org.firstinspires.ftc.teamcode.systems;




import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/* TODO: Make a ColorMatrix class for holding the color values for each color
    the new class will take min/max color values and distance then just have a match method to check
 */
public class ColorSensorSubSystem {
    private final RevColorSensorV3 colorSensor;
    private final TelemetryManager telemetryM;

    // TODO: instead of an array, use a matrix (ask Jeff)

    //                                                    redHigh, redLow, greenHigh, GreenLow, blueHigh, BlueLow
    private static final double[] greenArtifactColors = { 0.258,    0.172,   0.688,    0.516,    0.516,     0.43 };


    private double[] purpleArtifactColors = { 0.258,    0.172,   0.688,    0.516,    0.516,     0.43 };

    // Example thresholds — tweak these based on testing
//        if (distance > 45 && distance < 55 &&
//    r > 0.43 && r < 0.516 &&
//    b > 1.118 && b < 1.204 &&
//    g > 0.688 && g < 0.774) {
//        return DetectedColor.PURPLE;
//    }
//
//        else if (distance > 40 && distance < 48 &&
//    r > 0.172 && r < 0.258 &&
//    b > 0.43 && b < 0.516 &&
//    g > 0.516 && g < 0.688) {
//        return DetectedColor.GREEN;
//    }

    private static final double MAX_DISTANCE = 65;
    private static final double MIN_DISTANCE = 20;

    // TODO: Move this either into its own file or in the ColorMatrix class under common
    public enum DetectedColor {
        UNKNOWN,
        NONE,
        PURPLE,
        GREEN,

    }

    public ColorSensorSubSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensor.setGain(100);

    }

    public void update(){
        double red = colorSensor.getNormalizedColors().red;
        double blue = colorSensor.getNormalizedColors().blue;
        double green = colorSensor.getNormalizedColors().green;
        telemetryM.addData("Distance: ", colorSensor.getDistance(DistanceUnit.MM));
        telemetryM.addData("Color red: ", red);
        telemetryM.addData("color blue: ", blue);
        telemetryM.addData("color green: ", green);
        telemetryM.addData("gain", colorSensor.getGain());

    }

    public DetectedColor getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = colorSensor.getDistance(DistanceUnit.MM);
        if (MIN_DISTANCE < distance || MAX_DISTANCE > distance)
            return DetectedColor.NONE;

        if (isMatch(colors,
                greenArtifactColors[0],
                greenArtifactColors[1],
                greenArtifactColors[2],
                greenArtifactColors[3],
                greenArtifactColors[4],
                greenArtifactColors[4])) {
            return DetectedColor.GREEN;
        }

        if (isMatch(colors,
                purpleArtifactColors[0],
                purpleArtifactColors[1],
                purpleArtifactColors[2],
                purpleArtifactColors[3],
                purpleArtifactColors[4],
                purpleArtifactColors[4])) {
            return DetectedColor.PURPLE;

        }

        return DetectedColor.UNKNOWN;
    }

    private static boolean isMatch(
            NormalizedRGBA colors,
            double redHigh,
            double redLow,
            double greenHigh,
            double greenLow,
            double blueHigh,
            double blueLow) {
        float r = colors.red / colors.alpha;
        float g = colors.green / colors.alpha;
        float b = colors.blue / colors.alpha;

        return (g <= greenHigh && g >= greenLow
        && r <= redHigh && r >= redLow
        && b <= blueHigh && b >= blueLow);
    }
        // TODO Code to check color range
        /*
        Color values:
        PURPLE BALL -
        * Distance: 52-58
        * Red: 0.0057 - 0.0059
        * Blue: 0.0138
        * Green: 0.0083
        * gain: 8
        GREEN BALL -
        * Distance: 44-47
        * Red: 0.0023 - 0.0024
        * Blue: 0.0056
        * Green: 0.007
        * gain: 8
        */

        /*
        Color values:
        GREEN BALL -
        * Distance: 40      34-35
        * Red: 0.0046       0.0057
        * Blue: 0.0132      0.0172
        * Green: 0.0165     0.0218
        * gain: 12
        PURPLE BALL -
        * Distance: 64-66       47-50
        * Red: 0.0044           0.0075-0.0079
        * Blue: 0.0093          0.0181-0.0183
        * Green: 0.0071-0.0073  0.011-0.108
        * gain: 12
        */




    }
