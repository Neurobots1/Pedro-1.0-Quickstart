package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ColourSensorThread implements Runnable {
    private ColorSensor colorSensor;
    private volatile String detectedColor = "None"; // `volatile` ensures thread safety
    private boolean running = true;
    private ElapsedTime timer = new ElapsedTime();

    public ColourSensorThread(ColorSensor colorSensor) {
        this.colorSensor = colorSensor;
    }

    @Override
    public void run() {
        while (running) {
            if (timer.milliseconds() > 5) { // Run every 5ms
                detectColor();
                timer.reset();
            }
            try {
                Thread.sleep(5); // Avoid CPU overuse
            } catch (InterruptedException e) {
                running = false;
            }
        }
    }

    private void detectColor() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int proximity = colorSensor.alpha(); // ✅ Integrate distance sensor

        double totalIntensity = red + green + blue;
        if (totalIntensity == 0 || proximity < 200) { // 200 is an example threshold
            detectedColor = "None";
            return;
        }

        double normRed = (double) red / totalIntensity;
        double normGreen = (double) green / totalIntensity;
        double normBlue = (double) blue / totalIntensity;

        if (isMatch(normRed, normGreen, normBlue, 0.583, 0.294, 0.123)) {
            detectedColor = "Red";
        } else if (isMatch(normRed, normGreen, normBlue, 0.119, 0.266, 0.615)) {
            detectedColor = "Blue";
        } else if (isMatch(normRed, normGreen, normBlue, 0.396, 0.510, 0.095)) {
            detectedColor = "Yellow";
        } else {
            detectedColor = "None";
        }
    }

    public String getDetectedColor() {
        return detectedColor;
    }

    private boolean isMatch(double r, double g, double b, double targetR, double targetG, double targetB) {
        return Math.abs(r - targetR) < 0.1 &&
                Math.abs(g - targetG) < 0.1 &&
                Math.abs(b - targetB) < 0.1;
    }

    public void stop() {
        running = false;
    }
}
