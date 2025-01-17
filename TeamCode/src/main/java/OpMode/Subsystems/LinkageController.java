package OpMode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LinkageController {

    public enum Position {
        RETRACTED(100),
        EXTENDED(1300);

        private final int targetPosition;

        Position(int targetPosition) {
            this.targetPosition = targetPosition;
        }

        public int getTargetPosition() {
            return targetPosition;
        }
    }

    private final PIDController pidController;
    private final DcMotorEx extendoMotor;
    private final double ticksPerDegree = 537.7 / 360; // Encoder ticks per degree
    private final double f = 0.1; // Feedforward constant for gravity compensation
    private int targetPosition = 0; // Target position in encoder ticks
    private boolean isPIDEnabled = true; // Enable/disable PID control

    private double p = 0.005; // Proportional gain
    private double i = 0.0; // Integral gain
    private double d = 0.0; // Derivative gain

    private static final double AMPERAGE_THRESHOLD = 2; // Amperage threshold for spike detection (adjust as needed)
    private static final double DRIVE_PAST_RETRACTED_POSITION = -0.5; // Power to drive past retracted position (negative to retract)
    private boolean isZeroing = false; // Flag for zeroing process
    private boolean hasZeroed = false; // Flag to track if zeroing is complete

    public LinkageController(HardwareMap hardwareMap, String motorName, double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.pidController = new PIDController(p, i, d);
        this.extendoMotor = hardwareMap.get(DcMotorEx.class, motorName);

        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendoMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the motor direction
        extendoMotor.setDirection(DcMotor.Direction.REVERSE);  // Reverse the direction if needed
    }

    // Set new PID values
    public void setPIDValues(double p, double i, double d) {
        this.p = p;
        this.i = i;
        this.d = d;
        pidController.setPID(p, i, d);
    }

    // Enable or disable PID control
    public void setPIDEnabled(boolean enabled) {
        isPIDEnabled = enabled;
    }

    // Update method to control the motor with PID
    public void update() {
        if (isPIDEnabled && hasZeroed) {
            int currentPos = extendoMotor.getCurrentPosition();
            double pid = pidController.calculate(currentPos, targetPosition);
            double ff = Math.cos(Math.toRadians(targetPosition / ticksPerDegree)) * f;
            double power = pid + ff;
            power *= 0.3; // Scale the power to 50% of its calculated value
            extendoMotor.setPower(power);
        } else {
            extendoMotor.setPower(0); // Stop motor when PID is disabled or zeroing is not complete
        }
    }

    // Set the position (EXTENDED or RETRACTED)
    public void setPosition(Position position) {
        targetPosition = position.getTargetPosition();
    }

    // Get current position of the motor
    public int getCurrentPosition() {
        return extendoMotor.getCurrentPosition();
    }

    // Get target position
    public int getTargetPosition() {
        return targetPosition;
    }

    // Check if the motor is at the target position
    public boolean isAtTarget() {
        int currentPos = getCurrentPosition();
        return Math.abs(currentPos - targetPosition) < 100; // Tolerance for small errors
    }

    // Reset the encoder to zero
    public void resetEncoder() {
        extendoMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
    }

    // Set motor power directly (used for zeroing)
    public void setMotorPower(double power) {
        extendoMotor.setPower(power);
    }

    // Start the zeroing process by driving the motor past the retracted position
    public void zeroMotor() {
        if (!isZeroing && !hasZeroed) {
            isZeroing = true;
            // Drive the motor past the retracted position (negative power to retract)
            extendoMotor.setPower(DRIVE_PAST_RETRACTED_POSITION);
        }
    }

    // Check if amperage spike occurs and reset the encoder once detected
    public void checkForAmperageSpike() {
        if (isZeroing) {
            double currentAmperage = extendoMotor.getCurrent(CurrentUnit.AMPS); // Get current in amperes
            if (currentAmperage > AMPERAGE_THRESHOLD) {
                resetEncoder(); // Reset encoder when spike is detected
                isZeroing = false; // Stop zeroing process
                hasZeroed = true; // Mark as zeroed
                setPIDEnabled(true); // Enable PID control
            }
        }
    }

    // Check if the motor is in the EXTENDED position
    public boolean isExtended() {
        return Math.abs(getCurrentPosition() - Position.EXTENDED.getTargetPosition()) < 100; // Tolerance for small errors
    }

    // Check if the motor is in the RETRACTED position
    public boolean isRetracted() {
        return Math.abs(getCurrentPosition() - Position.RETRACTED.getTargetPosition()) < 100; // Tolerance for small errors
    }

}