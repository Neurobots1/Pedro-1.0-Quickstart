package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

public class IntakeMotor {
    private DcMotor intakeMotor;

    public IntakeMotor(DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
        // Set the motor direction to FORWARD or REVERSE based on your setup
        intakeMotor.setDirection(DcMotor.Direction.REVERSE); // Set the motor direction to forward
    }

    // Method to run the motor for intaking at full speed (positive power)
    public void intake() {
        intakeMotor.setPower(-1.0);  // Run motor at full speed forward for intake
    }

    // Method to run the motor for outtaking at half speed (negative power)
    public void outtake() {
        intakeMotor.setPower(0.7);  // Run motor at 3/4 speed in reverse for outtake
    }

    // Method to run the motor at half speed to keep intaked pieces inside
    public void slowOuttake() {
        intakeMotor.setPower(0.3);  // Run motor at half speed to keep pieces inside
    }

    // Method to stop the intake motor
    public void stop() {
        intakeMotor.setPower(0);  // Stop motor
    }
}
