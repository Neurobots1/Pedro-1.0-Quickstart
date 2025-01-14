package OpMode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Measure Encoder Ticks with Button", group = "Test")
public class EncoderTest extends OpMode {
    
    private DcMotorEx motor;
    private int initialTicks = 0;
    private int ticksPerRevolution = 0;
    private boolean isRotating = false;

    @Override
    public void init() {
        // Initialize the motor (make sure you use the correct name in HardwareMap)
        motor = hardwareMap.get(DcMotorEx.class, "extendoMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD); // Set the motor direction if necessary
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // Reset encoder count
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // Set the motor to run without encoder resets
    }

    @Override
    public void loop() {
        // Get the current encoder tick count
        int currentTicks = motor.getCurrentPosition();
        
        // Display the encoder ticks in telemetry
        telemetry.addData("Current Encoder Ticks", currentTicks);
        telemetry.addData("Ticks Per Revolution", ticksPerRevolution);

        // Do not apply power to the motor; this allows manual rotation
        motor.setPower(0);  // No power applied, so the motor will not resist your hand rotation
        
        // When you manually rotate the motor, this counts how many ticks you've gone through
        if (isRotating) {
            // Check if a full revolution is completed
            if (currentTicks - initialTicks >= ticksPerRevolution) {
                // Once you've rotated one full revolution, calculate the PPR
                ticksPerRevolution = currentTicks - initialTicks;
                telemetry.addData("Full Revolution Detected", true);
            }
        }
        
        // Start or stop the measurement process
        if (gamepad1.a) {
            // Press 'A' to mark the completion of one full revolution
            if (!isRotating) {
                initialTicks = currentTicks;  // Record the current tick count when revolution begins
                isRotating = true;  // Mark that rotation is in progress
                telemetry.addData("Measuring", "Rotate the motor one full revolution");
            } else {
                // Once you press 'A' again, the program calculates ticks per revolution
                ticksPerRevolution = currentTicks - initialTicks;
                isRotating = false;  // Stop the measurement process
                telemetry.addData("Measurement Complete", "Ticks per revolution calculated.");
            }
        }

        // Optionally, you can reset the process with the 'B' button
        if (gamepad1.b) {
            isRotating = false;
            initialTicks = 0;
            ticksPerRevolution = 0;
            telemetry.addData("Measurement", "Reset");
        }

        telemetry.update();
    }
}
