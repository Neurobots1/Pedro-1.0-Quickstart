package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeServos {

    private Servo IntakeServoRight;
    private Servo IntakeServoLeft;

    // Constructor to initialize servos
    public IntakeServos(Servo rightServo, Servo leftServo) {
        this.IntakeServoRight = rightServo;
        this.IntakeServoLeft = leftServo;

        // Scale the servos' range during initialization
        IntakeServoRight.scaleRange(0, 1);  // Scale for ExtendoServoRight (reversed) ( change this for adjustments )
        //IntakeServoLeft.scaleRange(0, 1);   // Scale for ExtendoServoLeft (non-reversed) ( change this for adjustments )
    }

    // Method to move both servos to the Transfer Position
    public void intakePosition() {
        IntakeServoRight.setPosition(0.25);  // Set to maximum after scaling
        //IntakeServoLeft.setPosition(0.9);   // Set to maximum after scaling
    }

    // Method to move both servos to the IntakeBoolean Position
    public void transferPosition() {
        IntakeServoRight.setPosition(1);  // Set to minimum after scaling
        //IntakeServoLeft.setPosition(0.0);   // Set to minimum after scaling
    }

    public void neutralPosition() {
        IntakeServoRight.setPosition(0.8);  // Set to minimum after scaling
        //IntakeServoLeft.setPosition(0.0);   // Set to minimum after scaling
    }

    // Method to check if the servos are in the IntakeBoolean Position
    public boolean isIntakePosition() {
        double tolerance = 0.1;
        boolean rightIntake = Math.abs(IntakeServoRight.getPosition() - 0.1) < tolerance;
        //boolean leftIntake = Math.abs(IntakeServoLeft.getPosition() - 0.9) < tolerance;
        return rightIntake /*&& leftIntake*/;
    }

    // Method to check if the servos are in the Transfer Position
    public boolean isTransferPosition() {
        double tolerance = 0.1;
        boolean rightTransfer = Math.abs(IntakeServoRight.getPosition() - 1.0) < tolerance;
        //boolean leftTransfer = Math.abs(IntakeServoLeft.getPosition() - 0.0) < tolerance;
        return rightTransfer /*&& leftTransfer*/;
    }
}
