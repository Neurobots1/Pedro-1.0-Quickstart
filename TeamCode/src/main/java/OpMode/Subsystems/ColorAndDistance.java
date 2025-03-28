package OpMode.Subsystems;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorAndDistance{
    private RevColorSensorV3 colorSensor;
    private String detectedColor = "None";
    private boolean objectDetected = false;

    public ColorAndDistance(RevColorSensorV3 colorSensor) {
        this.colorSensor = colorSensor;
    }



    public void update() {
        double distance = colorSensor.getDistance(DistanceUnit.MM);
        objectDetected = distance < 20;

          /* if (!objectDetected) {
            detectedColor = "Undetermined";
            return;
        } */


        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float[] hsv = new float[3];


        Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );

        detectedColor = determineColor(hsv[0], hsv[1], hsv[2]);
    }

    private String determineColor(float hue, float saturation, float value) {

        if (hue >= 20 && hue < 55) {
            return "Red"; // about 30
        } else if (hue >= 200 && hue < 300) {
            return "Blue"; // about 225
        } else if (hue >= 60 && hue < 100) {
            return "Yellow"; // about 60
        }

        return "None";
    }

    public String getDetectedColor() {
        return detectedColor;
    }

    public boolean isObjectDetected() {
        return objectDetected;
    }
}
