package OpMode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeTest {
    private DcMotorEx intakeMotor;
    private Servo intakeServoRight;
    private final Telemetry telemetry;
    public IntakeTest(Telemetry telemetry){this.telemetry = telemetry;}


    public void setup(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakemotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
    }


    // Meth;od to run the motor for intaking at full speed (positive power)
    public void intake() {
        intakeMotor.setPower(-1.0);  // Run motor at full speed forward for IntakeMotor
    }

    // Method to run the motor for outtaking at half speed (negative power)
    public void outtake() {
        intakeMotor.setPower(0.7);  // Run motor at 3/4 speed in reverse for outtake
    }

    // Method to run the motor at half speed to keep intaked pieces inside
    public void slowOuttakeMotor() {
        intakeMotor.setPower(0.3);  // Run motor at half speed to keep pieces inside
    }

    // Method to stop the IntakeMotor motor
    public void stop() {
        intakeMotor.setPower(0);  // Stop motor
    }

    // Method to move both servos to the Transfer Position
    public void intakePosition() {
        intakeServoRight.setPosition(0.8);  // Set to maximum after scaling
        //IntakeServoLeft.setPosition(0.9);   // Set to maximum after scaling
    }

    // Method to move both servos to the IntakeBoolean Position
    public void transferPosition() {
        intakeServoRight.setPosition(0.1);  // Set to minimum after scaling
        //IntakeServoLeft.setPosition(0.0);   // Set to minimum after scaling
    }


    // Method to check if the servos are in the IntakeBoolean Position
    public boolean isIntakePosition() {
        double tolerance = 0.1;
        boolean rightIntake = Math.abs(intakeServoRight.getPosition() - 0.1) < tolerance;
        //boolean leftIntake = Math.abs(IntakeServoLeft.getPosition() - 0.9) < tolerance;
        return rightIntake /*&& leftIntake*/;
    }

    // Method to check if the servos are in the Transfer Position
    public boolean isTransferPosition() {
        double tolerance = 0.1;
        boolean rightTransfer = Math.abs(intakeServoRight.getPosition() - 1.0) < tolerance;
        //boolean leftTransfer = Math.abs(IntakeServoLeft.getPosition() - 0.0) < tolerance;
        return rightTransfer /*&& leftTransfer*/;
    }
}

