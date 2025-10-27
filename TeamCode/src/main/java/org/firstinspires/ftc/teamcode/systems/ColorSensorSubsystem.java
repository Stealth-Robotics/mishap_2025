package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.SlotState;

@Configurable
public class ColorSensorSubsystem {
    private final RevColorSensorV3 colorSensor;

    private SlotState lastDetection = SlotState.UNKNOWN;
    private final TelemetryManager telemetryM;

    public static double[] greenArtifactColors =
//          redHigh, redLow, greenHigh, GreenLow, blueHigh, BlueLow
            {1.4,       0.2,    6.0,       2.0,    2.7,      .5};

    public static double[] purpleArtifactColors =
//          redHigh, redLow, greenHigh, GreenLow, blueHigh, BlueLow
            {2.5,       .4,    2.8,        1.0,    5.0,       2.4};

    public static double[] paddleWhiteIgnore =
//          redHigh, redLow, greenHigh, GreenLow, blueHigh, BlueLow
            {1,       .6,    2.0,        1.3,    2,       1.4};

    public static double[] paddleDistance =
            { 42, 50};

    private static final double MAX_DISTANCE = 75;
    private static final double MIN_DISTANCE = 20;

    //TODO: test code
    private double
            redMaxHigh = -1, redMinLow = 10,
            greenMaxHigh = -1, greenMinLow = 10,
            blueMaxHigh = -1, blueMinLow = 10,
            distanceMin = 5000, distanceMax = -1;


    public ColorSensorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensor.setGain(100);
    }

    // TODO: REMOVE once this is tested
    public void resetHighLow() {
        redMaxHigh = -1;
        redMinLow = 10;
        greenMaxHigh = -1;
        greenMinLow = 10;
        blueMaxHigh = -1;
        blueMinLow = 10;
        distanceMin = 5000;
        distanceMax = -1;
    }

    public void update(){
        lastDetection = getDetectedColor();
//
        telemetryM.addData("Detected color: ", lastDetection);
//        // TEST CODE
//        telemetryM.addData("Distance: ", colorSensor.getDistance(DistanceUnit.MM));
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        telemetryM.addData("Color red: ", colors.red / colors.alpha);
//        telemetryM.addData("color green: ", colors.green / colors.alpha);
//        telemetryM.addData("color blue: ", colors.blue / colors.alpha);
//        // TODO: this should be deleted once this is tested
//        telemetryM.addData("Red High: ", redMaxHigh);
//        telemetryM.addData("Red Low: ", redMinLow);
//        telemetryM.addData("Green High: ", greenMaxHigh);
//        telemetryM.addData("Green Low: ", greenMinLow);
//        telemetryM.addData("Blue High: ", blueMaxHigh);
//        telemetryM.addData("Blue Low: ", blueMinLow);
//        double dis = colorSensor.getDistance(DistanceUnit.MM);
//        if (dis > distanceMax)
//            distanceMax = dis;
//        if (dis < distanceMin)
//            distanceMin = dis;
//        telemetryM.addData("Distance Max: ", distanceMax);
//        telemetryM.addData("Distance Min: ", distanceMin);

    }

    private  SlotState getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        double distance = colorSensor.getDistance(DistanceUnit.MM);
        if (distance < MIN_DISTANCE || distance > MAX_DISTANCE) {
            return SlotState.EMPTY;
        }

        // Test for green artifact
        if (isMatch(colors,
                greenArtifactColors[0],
                greenArtifactColors[1],
                greenArtifactColors[2],
                greenArtifactColors[3],
                greenArtifactColors[4],
                greenArtifactColors[5])) {
            return SlotState.ARTIFACT_GREEN;
        }

        // Test for Purple Artifact
        if (isMatch(colors,
                purpleArtifactColors[0],
                purpleArtifactColors[1],
                purpleArtifactColors[2],
                purpleArtifactColors[3],
                purpleArtifactColors[4],
                purpleArtifactColors[5])) {
            return SlotState.ARTIFACT_PURPLE;

        }
        // ignore paddle movement
        if (distance >= paddleDistance[0] && distance <= paddleDistance[1]) {
            if (isMatch(colors,
                    paddleWhiteIgnore[0],
                    paddleWhiteIgnore[1],
                    paddleWhiteIgnore[2],
                    paddleWhiteIgnore[3],
                    paddleWhiteIgnore[4],
                    paddleWhiteIgnore[5])) {
                return SlotState.TRANSITIONING;
            }
        }
        return SlotState.UNKNOWN;
    }

    public SlotState getLastDetection() {
        return lastDetection;
    }


    private  boolean isMatch(
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

        //TODO: Test code for when the color sensor isn't reading well
//        if (r > this.redMaxHigh)
//            this.redMaxHigh = r;
//        if (r < this.redMinLow)
//            this.redMinLow = r;
//        if (g > this.greenMaxHigh)
//            this.greenMaxHigh = g;
//        if (g < this.greenMinLow)
//            this.greenMinLow = g;
//        if (b > this.blueMaxHigh)
//            this.blueMaxHigh = b;
//        if (b < this.blueMinLow)
//            this.blueMinLow = b;

        // END TEMP CODE

        return (
                (r <= redHigh && r >= redLow)
                && (g <= greenHigh && g >= greenLow)
                && (b <= blueHigh && b >= blueLow));
    }
}