package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class FlapServo {

    private Servo FlapServo;

    public FlapServo(Servo flapServo) {
        this.FlapServo = flapServo;

        // Scale the servos' range during initialization
        FlapServo.scaleRange(0, 0.2);  // Scale for ExtendoServoRight (reversed) ( change this for adjustments )
    }

    public void FlapClose(){
        FlapServo.setPosition(1);
    }

    public void FlapOpen(){
        FlapServo.setPosition(0);
    }

    public boolean isOpen() {
        // Use a small tolerance to check if the servo is approximately in the open position
        double tolerance = 0.1;
        return Math.abs(FlapServo.getPosition() - 1.0) < tolerance;
    }

    // Method to check if the servo is in the Closed position
    public boolean isClosed() {
        // Use a small tolerance to check if the servo is approximately in the closed position
        double tolerance = 0.1;
        return Math.abs(FlapServo.getPosition() - 0.0) < tolerance;
    }

}
