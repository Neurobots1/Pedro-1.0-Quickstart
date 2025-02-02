package OpMode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ViperSlides {
    public enum Target {
        GROUND(0),
        LOW(1400),
        MEDIUM(1900),
        HIGH(2950),
        LEVEL1(1000);

        private final int position;

        Target(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }

    private PIDController controller;
    private DcMotorEx slidemotorLeft;
    private DcMotorEx slidemotorRight;
    private TouchSensor limitSwitch;
    private final double ticksInDegree = 537.7 / 360;
    private final double f = 0.1;
    private int target = 0;
    private boolean isPIDEnabled = true;  // Tracks whether PID control is enabled

    public ViperSlides(DcMotorEx slidemotorLeft, DcMotorEx slidemotorRight, TouchSensor limitSwitch, double p, double i, double d) {
        this.controller = new PIDController(p, i, d);
        this.slidemotorLeft = slidemotorLeft;
        this.slidemotorRight = slidemotorRight;
        this.limitSwitch = limitSwitch;

        this.slidemotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.slidemotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slidemotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.slidemotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.slidemotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Method to enable/disable PID control
    public void setPIDEnabled(boolean enabled) {
        isPIDEnabled = enabled;
    }

    public void update() {
        if (isPIDEnabled) {
            int slidePosLeft = slidemotorLeft.getCurrentPosition();
            int slidePosRight = slidemotorRight.getCurrentPosition();
            double pidLeft = controller.calculate(slidePosLeft, target);
            double pidRight = controller.calculate(slidePosRight, target-35);
            double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
            double powerLeft = pidLeft + ff;
            double powerRight = pidRight + ff;

            slidemotorLeft.setPower(powerLeft);
            slidemotorRight.setPower(powerRight);
        } else {
            // Disable motor power control if PID is disabled (used for manual reset)
            slidemotorLeft.setPower(0);
            slidemotorRight.setPower(0);
        }
    }

    public void setTarget(Target target) {
        this.target = target.getPosition();
    }

    public int getSlidePositionLeft() {
        return slidemotorLeft.getCurrentPosition();
    }

    public int getSlidePositionRight() {
        return slidemotorRight.getCurrentPosition();
    }

    public int getSlidePosition() {
        return (slidemotorLeft.getCurrentPosition() + slidemotorRight.getCurrentPosition()) / 2;
    }

    public int getTarget() {
        return target;
    }

    public boolean isAtTargetPosition(Target target) {
        int currentPosition = getSlidePosition();
        return currentPosition >= target.getPosition() - 50 && currentPosition <= target.getPosition() + 50; // Tolerance for small errors
    }

    public boolean isLimitSwitchPressed() {
        return limitSwitch.isPressed();
    }

    public void resetPosition() {
        target = 0;  // Set target to 0 after reset, if necessary
        slidemotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidemotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setSlidePower(double power) {
        slidemotorLeft.setPower(power);
        slidemotorRight.setPower(power);
    }
}
