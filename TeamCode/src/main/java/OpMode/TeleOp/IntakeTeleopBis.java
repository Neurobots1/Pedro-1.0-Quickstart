package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;



import OpMode.Subsystems.ColourSensorThread;
import OpMode.Subsystems.IntakeMotor;

import com.qualcomm.robotcore.hardware.Gamepad;
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
    private DcMotor extendoMotor;
    private IntakeServos intakeServos; // IntakeBoolean subsystem instance

    boolean waitingForExtension = false;

    public boolean IntakeBoolean = false;

    private boolean previousLeftTriggerState = false;
    private boolean currentLeftTriggerState = false;

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
        linkageController.zeroMotor();

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();




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


        currentLeftTriggerState = gamepad1.a;
        if (currentLeftTriggerState && !previousLeftTriggerState) {

            if(IntakeBoolean){
                linkageController.setPosition(LinkageController.Position.EXTENDED);
                if (!detectedColor.equals("Blue")){
                    intakeMotor.intake();
                }
                intakeMotor.stop();
            } else {
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                intakeMotor.stop();
            }

            IntakeBoolean = !IntakeBoolean;
        }
        previousLeftTriggerState = currentLeftTriggerState;



        // Telemetry output
        telemetry.addData("Detected Color", detectedColor);
        telemetry.update();
    }

    @Override
    public void stop() {
        colourSensorThread.stop(); // Stop the color detection thread
    }
}
