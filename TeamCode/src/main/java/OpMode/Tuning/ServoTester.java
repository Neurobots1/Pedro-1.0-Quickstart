package OpMode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "ServoTester", group = "Test")
public class ServoTester extends OpMode {

    public static double servoPosition = 0.5; // Default position to 0.5 (center)
    public static boolean isReversed = false; // Toggle to reverse the servo direction
    public static int selectedServoIndex = 0; // Select the servo to test via Dashboard
    public static boolean servoActive = true; // Toggle for servo on/off from the Dashboard

    private List<String> servoNames = new ArrayList<>();
    private Servo currentServo;

    @Override
    public void init() {
        // Add your actual servo names here
        servoNames.add("IntakeServoRight");
        servoNames.add("IntakeServoLeft");
        servoNames.add("ExtendoServoRight");
        servoNames.add("ExtendoServoLeft");
        servoNames.add("BucketServoRight");
        servoNames.add("BucketServoLeft");
        servoNames.add("ClawServo");
        servoNames.add("FlapServo");

        updateSelectedServo(); // Set the initial servo
    }

    @Override
    public void loop() {
        // Ensure the selected index is valid
        if (!servoNames.isEmpty()) {
            selectedServoIndex = Math.abs(selectedServoIndex) % servoNames.size(); // Prevent index out-of-bounds
            updateSelectedServo();

            if (currentServo != null) {
                double actualPosition = isReversed ? 1.0 - servoPosition : servoPosition; // Apply reverse logic

                // Use the servoActive toggle to decide whether the servo moves
                if (servoActive) {
                    currentServo.setPosition(actualPosition); // Set the servo position when active
                } else {
                    currentServo.setPosition(0); // Set to 0 when inactive (or you can set to 1)
                }

                // Send telemetry data for feedback
                telemetry.addData("Testing Servo", servoNames.get(selectedServoIndex)); // Display meaningful name
                telemetry.addData("Input Position", servoPosition);
                telemetry.addData("Reversed", isReversed);
                telemetry.addData("Actual Servo Position", actualPosition);
                telemetry.addLine("Servo Behavior (Non-Reversed):");
                telemetry.addLine("  - 0.0: Counterclockwise (Left)");
                telemetry.addLine("  - 1.0: Clockwise (Right)");

                // Show all available servos in telemetry for reference
                telemetry.addLine("Available Servos:");
                for (int i = 0; i < servoNames.size(); i++) {
                    telemetry.addData("Servo " + i, servoNames.get(i));
                }

                // Show toggle status
                telemetry.addData("Servo Active", servoActive ? "ON" : "OFF");
            } else {
                telemetry.addLine("Selected servo not found in hardware map!");
            }
        } else {
            telemetry.addLine("No servos detected in hardware map!");
        }

        telemetry.update();
    }

    private void updateSelectedServo() {
        if (!servoNames.isEmpty() && selectedServoIndex < servoNames.size()) {
            currentServo = hardwareMap.get(Servo.class, servoNames.get(selectedServoIndex));
        } else {
            currentServo = null;
        }
    }
}
