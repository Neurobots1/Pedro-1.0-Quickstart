package OpMode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;



import OpMode.Subsystems.ColourSensorThread;
import OpMode.Subsystems.IntakeMotor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.GamePieceDetection;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServos;
import OpMode.Subsystems.ViperSlides;
import OpMode.Subsystems.LinkageController;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "IntakeTeleOp", group = "Active")
public class IntakeTeleopBis extends OpMode {

    private IntakeMotor intakeMotor;
    private ColourSensorThread colourSensorThread;
    private Thread colorThread;
    private LinkageController linkageController;

    private boolean Intake = false;

    @Override
    public void init() {
        // Initialize intake motor
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));

        Gamepad currentGamepad1 = new Gamepad();



        Gamepad previousGamepad1 = new Gamepad();

        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");




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

        if(Intake = true){
            linkageController.setPosition(LinkageController.Position.EXTENDED);
            while (!detectedColor.equals("Red")){
                intakeMotor.intake();
            }
            intakeMotor.stop();
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
