package OpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Measure Encoder Ticks", group = "Test")
public class MeasureEncoderTicks extends OpMode {
    
    private DcMotorEx motor;
    private int initialTicks = 0;
    private int ticksPerRevolution = 0;
    private boolean isRotating = false;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        // Initialize the motor (make sure you use the correct name in HardwareMap)
        motor = hardwareMap.get(DcMotorEx.class, "motor_name");
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
            // If you press 'A' button, start or reset the measurement process
            initialTicks = currentTicks;
            isRotating = true;
            telemetry.addData("Measuring", "Rotate the motor one full revolution");
        }
        
        if (gamepad1.b) {
            // Press 'B' to stop and reset the process (if needed)
            isRotating = false;
            initialTicks = 0;
            ticksPerRevolution = 0;
            telemetry.addData("Measurement", "Stopped");
        }

        telemetry.update();
    }
}
