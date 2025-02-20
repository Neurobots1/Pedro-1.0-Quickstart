package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;



import OpMode.Subsystems.ColourSensorThread;
import OpMode.Subsystems.IntakeMotor;

import com.qualcomm.robotcore.hardware.Servo;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.LinkageController;

@TeleOp(name = "IntakeTeleOpBis", group = "Active")
public class IntakeTeleopBis extends OpMode {


    // subsystem
    private LinkageController linkageController;
    private IntakeMotor intakeMotor;
    private ColourSensorThread colourSensorThread;
    private Thread colorThread;


    // Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServos intakeServos; // Intake subsystem instance

    private boolean Intake = false;

    @Override
    public void init() {
        // Initialize intake motor
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);

        // Initialize intake servos
        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);
        intakeServos.transferPosition(); // Set intake servos to transfer position


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

        if(gamepad1.a){
            Intake = !Intake;
        }

        if(Intake){
            linkageController.setPosition(LinkageController.Position.EXTENDED);
            while (!detectedColor.equals("Blue")){
                intakeMotor.intake();
            }
            intakeMotor.stop();
        } else {
            linkageController.setPosition(LinkageController.Position.RETRACTED);
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
