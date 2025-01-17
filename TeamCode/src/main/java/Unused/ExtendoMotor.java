package Unused;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ExtendoMotor {

    // Enum to represent different positions
    public enum Target {
        RETRACTED(150),  // Default retracted position
        EXTENDED(-800); // Default extended position

        private final int position;

        Target(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }
    private PIDController controller;
    private DcMotorEx extendoMotor;
    private final double f = 0.1;  // Feedforward constant
    private int target = Target.RETRACTED.getPosition();  // Default to retracted position
    private boolean isPIDEnabled = true;  // Tracks whether PID control is enabled

    public ExtendoMotor(DcMotorEx extendoMotor,double p, double i, double d) {
        // Initialize the PIDController with the predefined PID values
        this.controller = new PIDController(p, i, d);
        this.extendoMotor = extendoMotor;

        // Configure the motor
        extendoMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // Adjust direction if necessary
        extendoMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Method to enable/disable PID control
    public void setPIDEnabled(boolean enabled) {
        isPIDEnabled = enabled;
    }

    public void update() {
        if (isPIDEnabled) {
            int currentPosition = extendoMotor.getCurrentPosition();
            double pidOutput = controller.calculate(currentPosition, target);
            double ff = Math.cos(Math.toRadians(target)) * f;  // Feedforward term
            double power = pidOutput + ff;

            extendoMotor.setPower(power);
        } else {
            // Disable motor power control if PID is disabled (used for manual reset)
            extendoMotor.setPower(0);
        }
    }

    // Method to set the target position using the enum
    public void setTarget(Target target) {
        this.target = target.getPosition();
    }

    // Get the current motor position
    public int getCurrentPosition() {
        return extendoMotor.getCurrentPosition();
    }

    // Get the current target position
    public int getTarget() {
        return target;
    }

    // Check if the motor is at the target position
    public boolean isAtTargetPosition() {
        int currentPosition = getCurrentPosition();
        return currentPosition >= target - 50 && currentPosition <= target + 50; // Tolerance for small errors
    }

    // Reset the position of the motor
    public void resetPosition() {
        target = Target.RETRACTED.getPosition();  // Set target to 0 (retracted) after reset
        extendoMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendoMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Set the motor power directly (useful if not using PID)
    public void setMotorPower(double power) {
        extendoMotor.setPower(power);
    }

    // New methods to check if the motor is extended or retracted

    // Check if the motor is extended (within tolerance)
    public boolean isExtended() {
        return getCurrentPosition() >= Target.EXTENDED.getPosition() - 50 && getCurrentPosition() <= Target.EXTENDED.getPosition() + 50;
    }

    // Check if the motor is retracted (within tolerance)
    public boolean isRetracted() {
        return getCurrentPosition() >= Target.RETRACTED.getPosition() - 50 && getCurrentPosition() <= Target.RETRACTED.getPosition() + 50;
    }
}
