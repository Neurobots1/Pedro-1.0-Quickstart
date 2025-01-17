package OpMode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.GamePieceDetection;
import OpMode.Subsystems.ClawServo;
import Unused.ExtendoServos;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import OpMode.Subsystems.IntakeServos;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "BlueTeleopOld", group = "Inactive")
public class BlueTeleopOld extends OpMode {

    // Viper Slide Variables
    private PIDController controller;
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    public static int target = 0;
    private final double ticks_in_degree = 537.7 / 360;
    private DcMotorEx slidemotorright;
    private DcMotorEx slidemotorleft;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);
    private FtcDashboard dashboard;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;
    private boolean limitSwitchPreviouslyPressed = false;

    // Intake and Extendo Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServos intakeServos; // Intake subsystem instance
    private Servo extendoServoRight;
    private Servo extendoServoLeft;
    private ExtendoServos extendoServos;
    private ClawServo clawServo;

    // Intake Motor and Color Sensor
    private DcMotor intakemotor;
    private ColorSensor colorSensor;
    private GamePieceDetection gamePieceDetection;
    private boolean hasRumbled = false;

    // Loop Timer
    private ElapsedTime loopTimer;
    // Declare the timer for the extendo servo retraction
    private ElapsedTime retractTimer = new ElapsedTime();
    private boolean isRetracting = false;
    // Declare the timer for retract button action
    private ElapsedTime retractButtonTimer = new ElapsedTime();
    private boolean isRetractingButtonPressed = false;


    @Override
    public void init() {
        // Initialize GamePieceDetection
        gamePieceDetection = new GamePieceDetection(hardwareMap.get(ColorSensor.class, "colorSensor"));

        // Initialize the loop timer
        loopTimer = new ElapsedTime();

        // Initialize Viper Slide
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize Pedro follower
        Constants.setConstants(FConstants.class,LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Initialize slide motors
        slidemotorleft = hardwareMap.get(DcMotorEx.class, "slidemotorleft");
        slidemotorleft.setDirection(DcMotorSimple.Direction.REVERSE);
        slidemotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slidemotorright = hardwareMap.get(DcMotorEx.class, "slidemotorright");
        slidemotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidemotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize REV Touch Sensor
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        // Initialize motors and sensors
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Initialize intake servos
        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServos(intakeServoRight , intakeServoLeft);

        // Initialize extendo servos
        extendoServoRight = hardwareMap.get(Servo.class, "ExtendoServoRight");
        extendoServoLeft = hardwareMap.get(Servo.class, "ExtendoServoLeft");
        extendoServos = new ExtendoServos(extendoServoRight, extendoServoLeft);

        // Set extendo servos to retracted position
        extendoServos.retract();

        // Set intake servos to transfer position
        intakeServos.transferPosition();
    }

    @Override
    public void start() {
        // Ensure the follower starts TeleOp drive
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // TeleOp movement
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        // Game Piece Detection and Rumble Feedback
        gamePieceDetection.detectColor();
        String detectedColor = gamePieceDetection.getDetectedColor();
        if ((detectedColor.equals("Blue") || detectedColor.equals("Yellow")) && !hasRumbled) {
            gamepad1.rumble(1000);  // Rumble for 1 second
            hasRumbled = true;
        }
        if (!detectedColor.equals("Blue") && !detectedColor.equals("Yellow")) {
            hasRumbled = false;
        }

        // Opponent Color Detection (e.g., Red)
        if (detectedColor.equals("Red")) {
            intakemotor.setPower(0.5);  // Outtake at low power
        } else if (gamepad1.left_bumper) {
            intakemotor.setPower(-1.0);  // Intake
        } else if (gamepad1.right_bumper) {
            intakemotor.setPower(0.5);  // Outtake
        } else {
            intakemotor.setPower(0);  // Stop intake motor
        }

        // Servo Control with 700ms Timer for Retraction
        if (gamepad1.dpad_right) {
            if (extendoServos.isExtended()) {
                intakeServos.intakePosition();
            } else {
                telemetry.addData("Warning", "Cannot move intake servos to intaking position while extendo servos are retracted!");
            }
        } else if (gamepad1.dpad_down) {
            if (!isRetracting) {
                intakeServos.transferPosition();  // Move intake servos to transfer position
                retractTimer.reset();
                isRetracting = true;  // Start the retraction process
            }
        } else if (gamepad1.dpad_up) {
            if (!isRetracting) {
                extendoServos.extend();
            }
        }

        // Manage the 700ms delay
        if (isRetracting && retractTimer.milliseconds() > 700) {
            extendoServos.retract();  // Ensure servos are fully retracted after 700ms
            isRetracting = false;    // Reset the state
        }

        // Viper Slide Control (PID)
        controller.setPID(p, i, d);
        int slidePosLeft = slidemotorleft.getCurrentPosition();
        int slidePosRight = slidemotorright.getCurrentPosition();
        double pidLeft = controller.calculate(slidePosLeft, target);
        double pidRight = controller.calculate(slidePosRight, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double powerLeft = pidLeft + ff;
        double powerRight = pidRight + ff;

        slidemotorleft.setPower(powerLeft);
        slidemotorright.setPower(powerRight);

        // Button presses to change target for the Viper slide
        if (gamepad1.y) {
            target = 2950;
        }
        if (gamepad1.a) {
            target = 0;
        }
        if (gamepad1.x) {
            target = 900;
        }
        if (gamepad1.b) {
            target = 1400;
        }

        // **Limit Switch Functionality (Debounce)**
        boolean isLimitSwitchPressed = limitSwitch.isPressed();
        if (isLimitSwitchPressed && !limitSwitchPreviouslyPressed) {
            // Reset encoders to zero
            slidemotorleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slidemotorright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Set target position to 0
            slidemotorleft.setTargetPosition(0);
            slidemotorright.setTargetPosition(0);

            // Re-enable running without encoders after reset
            slidemotorleft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slidemotorright.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            telemetry.addData("Limit Switch", "Resetting encoders");
        }

        // Update the state variable
        limitSwitchPreviouslyPressed = isLimitSwitchPressed;

        // Telemetry for debugging and visualization
        telemetry.addData("posLeft", slidePosLeft);
        telemetry.addData("posRight", slidePosRight);
        telemetry.addData("target", target);
        telemetry.addData("Detected Color", detectedColor);
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
