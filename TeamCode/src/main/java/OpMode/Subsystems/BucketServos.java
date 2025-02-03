package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class BucketServos {

    private Servo bucketServoRight;
    private Servo bucketServoLeft;

    public BucketServos(Servo rightServo, Servo leftServo) {
        this.bucketServoRight = rightServo;
        this.bucketServoLeft = leftServo;

        // Scale the servos' range during initialization
        bucketServoRight.scaleRange(0.2, 0.8);  // Adjust range for right servo
        //bucketServoLeft.scaleRange(0, 1);   // Adjust range for left servo
    }

    public void transferPosition() {
        bucketServoRight.setPosition(1);   // Transfer position for right servo
        //bucketServoLeft.setPosition(1);    // Transfer position for left servo
    }

    public void depositPosition() {
        bucketServoRight.setPosition(0);   // Deposit position for right servo
        //bucketServoLeft.setPosition(0);    // Deposit position for left servo
    }

    // Method to check if the servos are in the Transfer Position
    public boolean isTransferPosition() {
        double tolerance = 0.1;
        boolean rightTransfer = Math.abs(bucketServoRight.getPosition() - 1) < tolerance;
        //boolean leftTransfer = Math.abs(bucketServoLeft.getPosition() - 1.0) < tolerance;
        return rightTransfer /*&& leftTransfer*/;
    }

    // Method to check if the servos are in the Deposit Position
    public boolean isDepositPosition() {
        double tolerance = 0.1;
        boolean rightDeposit = Math.abs(bucketServoRight.getPosition() - 0) < tolerance;
        //boolean leftDeposit = Math.abs(bucketServoLeft.getPosition() - 0.0) < tolerance;
        return rightDeposit /*&& leftDeposit*/;
    }
}
