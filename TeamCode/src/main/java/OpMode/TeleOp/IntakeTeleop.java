package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import OpMode.Subsystems.ColourSensorThread;
import OpMode.Subsystems.IntakeMotor;

@TeleOp(name = "IntakeTeleOp", group = "Active")
public class IntakeTeleop extends OpMode {

    private IntakeMotor intakeMotor;
    private ColourSensorThread colourSensorThread;
    private Thread colorThread;

    @Override
    public void init() {
        // Initialize intake motor
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));

        // Initialize color sensor and thread
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        colourSensorThread = new ColourSensorThread(colorSensor);
        colorThread = new Thread(colourSensorThread);
        colorThread.start(); // Start the color detection thread

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Get detected color from the background thread
        String detectedColor = colourSensorThread.getDetectedColor();

        // Auto-reject red game elements
        if (detectedColor.equals("Red")) {
            intakeMotor.outtake(); // Auto outtake when detecting red
        } else if (gamepad1.left_bumper) {
            intakeMotor.intake(); // Manual intake
        } else if (gamepad1.right_bumper) {
            intakeMotor.outtake(); // Manual outtake
        } else {
            intakeMotor.stop(); // Stop intake motor
        }

        // Telemetry output
        telemetry.addData("Detected Color", detectedColor);
        telemetry.update();
    }

    @Override
    public void stop() {
        colourSensorThread.stop(); // Stop the color detection thread
    }
}
