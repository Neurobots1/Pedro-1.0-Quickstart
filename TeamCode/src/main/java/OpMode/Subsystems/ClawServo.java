package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServo {

    private Servo ClawServo;

    // Constructor to initialize servos
    public ClawServo(Servo clawServo) {
        this.ClawServo = clawServo;

        // Scale the servos' range during initialization
        ClawServo.scaleRange(0.40, 0.75);  // Scale for ClawServo (change this for adjustments)
    }

    // Method to move both servos to the Transfer Position
    public void openPosition() {
        ClawServo.setPosition(0);  // Set to maximum after scaling
    }

    // Method to move both servos to the Intake Position
    public void closedPosition() {
        ClawServo.setPosition(1);  // Set to minimum after scaling
    }

    // Method to check if the servo is in the Open position
    public boolean isOpen() {
        // Use a small tolerance to check if the servo is approximately in the open position
        double tolerance = 0.1;
        return Math.abs(ClawServo.getPosition() - 0.0) < tolerance;
    }

    // Method to check if the servo is in the Closed position
    public boolean isClosed() {
        // Use a small tolerance to check if the servo is approximately in the closed position
        double tolerance = 0.1;
        return Math.abs(ClawServo.getPosition() - 1.0) < tolerance;
    }
}
