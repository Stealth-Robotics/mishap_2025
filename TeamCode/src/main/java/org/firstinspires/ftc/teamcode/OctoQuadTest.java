package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.PanelsConfig;
import com.bylazar.panels.json.PanelsWidget;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.frozenmilk.util.graph.Graph;

@TeleOp(name="Octo Test", group="Tests")
@Disabled
public class OctoQuadTest extends OpMode {

    private static final int REV_PWM_1 = 4;
    private static final int REV_PWM_LOW = 1;
    private static final int REV_PWM_HIGH = 1024;
    private static final double DEGREES_PER_US = (360.0 / 1024.0);  // REV Through Bore Encoder
    private static final int    VELOCITY_SAMPLE_INTERVAL_MS = 25;   // To provide 40 updates/Sec.
    private static final double VELOCITY_SAMPLES_PER_S = (1000.0 / VELOCITY_SAMPLE_INTERVAL_MS);
    private OctoQuad octoquad;
    private TelemetryManager telemetryM;
    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        initOctoQuad();
    }

    private void initOctoQuad() {
        octoquad = hardwareMap.get(OctoQuad.class, "octoquad");
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.BANK1_QUADRATURE_BANK2_PULSE_WIDTH);
        octoquad.setAllVelocitySampleIntervals(VELOCITY_SAMPLE_INTERVAL_MS);

        // set channels 4 to 7 for REV through bore PWM (absolute) encoders
        for (int i = 4; i < 8; i++) {
            // false means the encoder will wrap back around to min value after reaching the max value
            octoquad.setSingleChannelPulseWidthTracksWrap(i, false);
            octoquad.setSingleChannelPulseWidthParams(i, REV_PWM_LOW, REV_PWM_HIGH);
            octoquad.setSingleEncoderDirection(i, OctoQuad.EncoderDirection.REVERSE);
        }

        // if the octoquad has a brown out this will make sure it picks the setttings back up.
        octoquad.saveParametersToFlash();
    }

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {

        telemetryM.addData("OctoQuad Version:", octoquad.getFirmwareVersionString());

        OctoQuad.EncoderDataBlock dataBlock = octoquad.readAllEncoderData();
        int dataPos = dataBlock.positions[REV_PWM_1];

        telemetryM.addData("Position", dataPos);
        double angle = dataPos * DEGREES_PER_US;
        telemetryM.addData("AngleNormalized", AngleUnit.normalizeDegrees(angle));
        telemetryM.addData("Angle", angle);

        double velocity = dataBlock.velocities[REV_PWM_1];
        telemetryM.addData("Velocity", velocity);


        telemetryM.update(telemetry);
    }
}

