package org.firstinspires.ftc.teamcode.Systems;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorSubSystem {
    RevColorSensorV3 colorSensor;
    Telemetry telemetry;
    TelemetryManager telemetryM;



    //private final Color kGreenTarget = new Color().green();

   // private ColorMatch

    public enum DetectedColor {
        RED,
        GREEN,
        BLUE
    }

    public ColorSensorSubSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        this.telemetry = telemetry;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        colorSensor.setGain(4);

        /*
        purple- (with gain 4)
            distance: 27.3983
            red: 0.0053
            green: 0.0069
            blue: 0.014

         green-(with gain 4)
            distance:31.844
            red: 0.0021
            green: 0.0081
            blue: 0.0067
         */


    }

    public void update(){
        double red = colorSensor.getNormalizedColors().red;
        double blue = colorSensor.getNormalizedColors().blue;
        double green = colorSensor.getNormalizedColors().green;
        double distance_sensor = colorSensor.getDistance(DistanceUnit.MM);
        telemetry.addData("Distance: ", colorSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Color red: ", red);
        telemetry.addData("color blue: ", blue);
        telemetry.addData("color green: ", green);
        telemetry.addData("gain", colorSensor.getGain());

//0.0009-red

    }
//    public DetectedColor getdetectcolor(){
//
//
//
//    }

    //Instructions:


}
