package org.firstinspires.ftc.teamcode.systems;


import android.graphics.ColorMatrix;

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
    RevColorSensorV3 colorSensor;
    Telemetry telemetry;
    TelemetryManager telemetryM;

    // TODO: instead of an array, use a matrix (ask Jeff)

    // RED, BLUE, GREEN, Distance
    //private double[] greanArtifactColorsHigh = {0.0023, 0.011, 0.0137, 40};

    // TODO: Move this either into its own file or in the ColorMatrix class under common
    public enum DetectedColor {
        UNKNOWN,
        NONE,
        PURPLE,
        GREEN,

    }

    public ColorSensorSubSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        this.telemetry = telemetry;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensor.setGain(12);

    }

    public void update(){
        double red = colorSensor.getNormalizedColors().red;
        double blue = colorSensor.getNormalizedColors().blue;
        double green = colorSensor.getNormalizedColors().green;
        telemetry.addData("Distance: ", colorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Color red: ", red);
        telemetry.addData("color blue: ", blue);
        telemetry.addData("color green: ", green);
        telemetry.addData("gain", colorSensor.getGain());

//0.0009-red

    }

    public DetectedColor getDetectedColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
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

        return DetectedColor.UNKNOWN;




    }

}
