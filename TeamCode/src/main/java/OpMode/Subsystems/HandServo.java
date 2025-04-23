package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class HandServo {

    private Servo HandServo;

    // Constructor to initialize servos
    public HandServo(Servo clawServo) {
        this.HandServo = clawServo;

        // Scale the servos' range during initialization
        HandServo.scaleRange(0, 1);  // Scale for ClawServo (change this for adjustments)
    }

    // Method to move both servos to the Transfer Position
    public void openPosition() {
        HandServo.setPosition(0.069);  // Set to maximum after scaling
    }

    // Method to move both servos to the IntakeBoolean Position
    public void closedPosition() {
        HandServo.setPosition(0.3);  // Set to minimum after scaling
    }

    // Method to check if the servo is in the Open position
    public boolean isOpen() {
        // Use a small tolerance to check if the servo is approximately in the open position
        double tolerance = 0.1;
        return Math.abs(HandServo.getPosition() - 0.0) < tolerance;
    }

    // Method to check if the servo is in the Closed position
    public boolean isClosed() {
        // Use a small tolerance to check if the servo is approximately in the closed position
        double tolerance = 0.1;
        return Math.abs(HandServo.getPosition() - 1.0) < tolerance;
    }
}
