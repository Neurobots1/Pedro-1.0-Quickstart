package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class ExtendoServos {

    private Servo extendoServoRight;
    private Servo extendoServoLeft;

    // Constructor to initialize servos
    public ExtendoServos(Servo rightServo, Servo leftServo) {
        this.extendoServoRight = rightServo;
        this.extendoServoLeft = leftServo;

        // Scale the servos' range during initialization
        extendoServoRight.scaleRange(0.72, 0.99);  // Scale for ExtendoServoRight (reversed) ( change this for adjustments )
        extendoServoLeft.scaleRange(0.03, 0.3);   // Scale for ExtendoServoLeft (non-reversed) ( change this for adjustments )
    }

    public boolean isExtended() {
        // Use a small tolerance to check if the servos are approximately in the extended position
        double tolerance = 0.1;
        boolean rightExtended = Math.abs(extendoServoRight.getPosition() - 0.0) < tolerance;
        boolean leftExtended = Math.abs(extendoServoLeft.getPosition() - 1.0) < tolerance;
        return rightExtended && leftExtended;
    }

    public boolean isRetracted() {
        // Check if servos are in the retracted position
        double tolerance = 0.1;
        boolean rightRetracted = Math.abs(extendoServoRight.getPosition() - 1.0) < tolerance;
        boolean leftRetracted = Math.abs(extendoServoLeft.getPosition() - 0.0) < tolerance;
        return rightRetracted && leftRetracted;
    }

    // Method to extend both servos
    public void extend() {
        extendoServoRight.setPosition(0);  // Set to maximum after scaling
        extendoServoLeft.setPosition(1.0);   // Set to maximum after scaling
    }

    // Method to retract both servos
    public void retract() {
        extendoServoRight.setPosition(1);  // Set to minimum after scaling
        extendoServoLeft.setPosition(0.0);   // Set to minimum after scaling
    }
}