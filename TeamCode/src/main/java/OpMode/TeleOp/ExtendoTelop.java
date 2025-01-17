package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import Unused.ExtendoMotor;

@TeleOp
public class ExtendoTelop extends OpMode {

    // Declare ExtendoMotor instance
    private ExtendoMotor extendoMotor;

    @Override
    public void init() {
        // Initialize ExtendoMotor with PID values (use default or customize as needed)
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "extendoMotor");  // Get motor from hardware map
        extendoMotor = new ExtendoMotor(motor, 0.005, 0, 0);  // Pass PID constants (example values)

        // Reset motor encoder position to zero
        extendoMotor.resetPosition();  // Reset encoder to 0

        // Set the motor target to the retracted position at the start
        extendoMotor.setTarget(ExtendoMotor.Target.RETRACTED);  // Set target to retracted (0)
    }

    @Override
    public void start() {
        // Ensure the motor starts in the retracted position
        extendoMotor.setTarget(ExtendoMotor.Target.RETRACTED);  // Set target to retracted position
    }

    @Override
    public void loop() {
        // Control the ExtendoMotor with D-pad
        if (gamepad1.dpad_up) {
            extendoMotor.setTarget(ExtendoMotor.Target.EXTENDED);  // Set motor to extended position
        } else if (gamepad1.dpad_down) {
            extendoMotor.setTarget(ExtendoMotor.Target.RETRACTED);  // Set motor to retracted position
        }

        // Update motor based on target
        extendoMotor.update();

        // Optional telemetry to see the current position of the motor
        telemetry.addData("Current Position", extendoMotor.getCurrentPosition());
        telemetry.addData("Target Position", extendoMotor.getTarget());
        telemetry.update();
    }
}
