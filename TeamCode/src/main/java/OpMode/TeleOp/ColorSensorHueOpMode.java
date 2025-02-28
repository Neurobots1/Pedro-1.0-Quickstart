package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import android.graphics.Color;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ColorSensorHueOpMode", group="TeleOp")
public class ColorSensorHueOpMode extends OpMode {
    private RevColorSensorV3 colorSensor;

    @Override
    public void init() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Get the distance from the color sensor
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        // Read color sensor values
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];

        // Convert RGB to HSV
        Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        // Print hue value and distance
        telemetry.addData("Hue", hsv[0]);
        telemetry.addData("Distance (mm)", distance);
        telemetry.update();
    }
}
