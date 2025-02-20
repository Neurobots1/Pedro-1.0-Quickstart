package OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ColourSensorThread;
import OpMode.Subsystems.GamePieceDetection;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;

@TeleOp(name = "NewIntakeTeleOp", group = "Active")
public class NewIntakeTeleop extends OpMode {

    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;
    // PedroPathing Teleop
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private FtcDashboard dashboard;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;
    private boolean wasLimitSwitchPressed = false;

    // Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServos intakeServos; // Intake subsystem instance
    private ClawServo clawServo;
    private Servo bucketServoRight;
    private Servo bucketServoLeft;
    private BucketServos bucketServos;

    // Intake Motor and Color Sensor
    private DcMotor intakemotor;
    private IntakeMotor intakeMotor;
    private DcMotor extendoMotor;
    private ColorSensor colorSensor;
    private GamePieceDetection gamePieceDetection;
    private boolean hasRumbled = false;

    // Loop Timer
    private ElapsedTime loopTimer;

    // Declare the LinkageController instance
    private LinkageController linkageController;

    // Declare the timer for the linkage retraction
    boolean waitingForExtension = false;


    private ColourSensorThread colourSensorThread;
    private Thread colorThread;
    private boolean Intake = false;

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

        // Initialize motors and sensors
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize intake servos
        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);
        intakeServos.transferPosition(); // Set intake servos to transfer position
        //Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");

        linkageController.zeroMotor();

    }

    @Override
    public void loop() {
        // Get detected color from the background thread
        String detectedColor = colourSensorThread.getDetectedColor();

        if (gamepad1.a){
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
