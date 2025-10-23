package org.firstinspires.ftc.robotcontroller.external.samples;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "Color Sensor ", group = "Sensor")
public class color_sensor extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    private static final float HIGH_GAIN = 45f;


    private static final float[] REF_GREEN = {0.191f, 0.454f, 0.355f};

    private static final float[] REF_PURPLE = {0.243f, 0.388f, 0.368f};

    private static final float CHROMA_MATCH_THRESH = 0.08f;

    private float baselineSum = 0f;
    private static final float SUM_BUMP = 1.25f;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colorSensor.setGain(HIGH_GAIN);

        telemetry.addLine("Calibrating background... hold nothing in front of the sensor");
        telemetry.addData("Fixed Gain", HIGH_GAIN);
        telemetry.update();

        int samples = 0;
        double sumAccum = 0.0;
        while (!isStarted() && !isStopRequested()) {
            NormalizedRGBA c = colorSensor.getNormalizedColors();
            double s = c.red + c.green + c.blue;
            sumAccum += s;
            samples++;

            telemetry.addLine("Calibrating...");
            telemetry.addData("Init Samples", samples);
            telemetry.addData("Current Sum", "%.3f", s);
            telemetry.update();
        }
        if (samples > 0) baselineSum = (float)(sumAccum / samples);
        else baselineSum = 0.12f;

        telemetry.addLine("Calibration complete");
        telemetry.addData("Baseline (R+G+B)", "%.3f", baselineSum);
        telemetry.addData("Min detect sum", "%.3f", baselineSum * SUM_BUMP);
        telemetry.update();

        waitForStart();

        float[] hsv = new float[3];

        while (opModeIsActive()) {
            NormalizedRGBA c = colorSensor.getNormalizedColors();
            float r = c.red, g = c.green, b = c.blue;
            float sum = r + g + b;

            // Default result
            String label = "None";

            if (sum >= baselineSum * SUM_BUMP) {
                float inv = (sum > 1e-6f) ? (1f / sum) : 0f;
                float cr = r * inv, cg = g * inv, cb = b * inv;

                float dGreen  = euclidean(cr, cg, cb, REF_GREEN[0],  REF_GREEN[1],  REF_GREEN[2]);
                float dPurple = euclidean(cr, cg, cb, REF_PURPLE[0], REF_PURPLE[1], REF_PURPLE[2]);

                if (dGreen < dPurple && dGreen <= CHROMA_MATCH_THRESH) {
                    label = "Green";
                } else if (dPurple < dGreen && dPurple <= CHROMA_MATCH_THRESH) {
                    label = "Purple";
                } else {
                    label = "Unknown";
                }
            }

            Color.colorToHSV(c.toColor(), hsv);

            telemetry.addLine("RGB(A)")
                    .addData("R", "%.3f", r)
                    .addData("G", "%.3f", g)
                    .addData("B", "%.3f", b)
                    .addData("A", "%.3f", c.alpha);
            telemetry.addLine("HSV")
                    .addData("H", "%.1f", hsv[0])
                    .addData("S", "%.3f", hsv[1])
                    .addData("V", "%.3f", hsv[2]);
            telemetry.addData("Sum R+G+B", "%.3f", sum);
            telemetry.addData("Baseline", "%.3f", baselineSum);
            telemetry.addData("Decision", label);
            telemetry.update();
        }
    }

    private float euclidean(float r1, float g1, float b1, float r2, float g2, float b2) {
        float dr = r1 - r2, dg = g1 - g2, db = b1 - b2;
        return (float)Math.sqrt(dr*dr + dg*dg + db*db);
    }
}
