package OpMode.TeleOp;

import static OpMode.Autonomous.AutonomousFSM.finalPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import OpMode.Subsystems.BucketServos;
import OpMode.Subsystems.ClawServo;
import OpMode.Subsystems.ColorAndDistance;
import OpMode.Subsystems.HandServo;
import OpMode.Subsystems.IntakeMotor;
import OpMode.Subsystems.IntakeServosNEW;
import OpMode.Subsystems.LinkageController;
import OpMode.Subsystems.ViperSlides;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "RedTeleopNEWER", group = "0")
public class RedTeleopNEWER extends OpMode {

    public enum IntakeState {
        INTAKE_START,
        INTAKE_EXTEND,
        INTAKE_RETRACT,
        OUTAKE_HUMAIN,
        OUTAKE_BLOCK,

        INTAKE_WAITFORBLOCK

    }


    IntakeState intakeState = IntakeState.INTAKE_START;
    ElapsedTime intakeTimer = new ElapsedTime();

    // Viper Slide Variables
    public static double p = 0.01, i = 0, d = 0.0;
    public static double f = 0.1;
    private ViperSlides viperSlides;
    // PedroPathing Teleop
    private Follower follower;
    private final Pose startPose = new Pose(finalPose.getX(), finalPose.getY(), finalPose.getHeading());
    private FtcDashboard dashboard;

    // REV Touch Sensor (Limit Switch)
    private TouchSensor limitSwitch;
    private boolean wasLimitSwitchPressed = false;

    // Servos
    private Servo intakeServoRight;
    private Servo intakeServoLeft;
    private IntakeServosNEW intakeServos;
    private ClawServo clawServo;
    private Servo bucketServoRight;
    private Servo bucketServoLeft;
    private BucketServos bucketServos;

    // Intake Motor and Color Sensor
    private DcMotor intakemotor;
    private IntakeMotor intakeMotor;
    private DcMotor extendoMotor;
    private ColorSensor colorSensor;
    private ColorAndDistance colorAndDistance;

    private HandServo handServo;


    private boolean hasRumbled = false;
    private ElapsedTime loopTimer;
    private LinkageController linkageController;
    boolean waitingForExtension = false;

    // Left Trigger Rising Edge Detection
    private boolean previousLeftTriggerState = false;
    private boolean currentLeftTriggerState = false;
    private boolean isClawOpen = true;

    @Override
    public void init() {

        // Initialize the loop timer
        loopTimer = new ElapsedTime();
        intakeTimer.reset();

        //startPose
        telemetry.addData("startPoseX", startPose.getX());
        telemetry.addData("startPoseY", startPose.getY());
        telemetry.addData("startPoseHeading", startPose.getHeading());

        // Corrected Color Sensor Initialization
        colorAndDistance = new ColorAndDistance(hardwareMap.get(RevColorSensorV3.class, "colorSensor"));

        // Initialize Viper Slide
        viperSlides = new ViperSlides(
                hardwareMap.get(DcMotorEx.class, "slidemotorleft"),
                hardwareMap.get(DcMotorEx.class, "slidemotorright"),
                hardwareMap.get(TouchSensor.class, "limitSwitch"),
                p, i, d
        );

        // Initialize Pedro follower
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();

        // Initialize Dashboard
        dashboard = FtcDashboard.getInstance();

        // Initialize REV Touch Sensor
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        // Initialize motors and sensors
        intakeMotor = new IntakeMotor(hardwareMap.get(DcMotor.class, "intakemotor"));

        // Initialize intake servos
        intakeServoRight = hardwareMap.get(Servo.class, "IntakeServoRight");
        intakeServoLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
        intakeServos = new IntakeServosNEW(intakeServoRight, intakeServoLeft);
        intakeServos.transferPosition();

        // Linkage
        linkageController = new LinkageController(hardwareMap, "extendoMotor", 0.005, 0.0, 0.0);
        telemetry.addData("Status", "Initialized");
        linkageController.zeroMotor();

        // Initialize claw servo
        clawServo = new ClawServo(hardwareMap.get(Servo.class, "ClawServo"));

        // Initialize Bucket Servo
        bucketServoRight = hardwareMap.get(Servo.class, "BucketServoRight");
        bucketServoLeft = hardwareMap.get(Servo.class, "BucketServoLeft");
        bucketServos = new BucketServos(bucketServoRight, bucketServoLeft);
        bucketServos.transferPosition();

        //Initialize Hand
        handServo = new HandServo(hardwareMap.get(Servo.class, "HandServo"));
        handServo.closedPosition();

    }

    @Override
    public void loop() {


        switch (intakeState) {


            case INTAKE_START:
                linkageController.setPosition(LinkageController.Position.RETRACTED);
                intakeMotor.stop();
                intakeServos.transferPosition();
                intakeTimer.reset();
                if (gamepad1.dpad_up) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                if (gamepad1.dpad_left) {
                    intakeMotor.outtake();
                }
                break;

            case INTAKE_EXTEND:
                linkageController.setPosition(LinkageController.Position.EXTENDED);
                if (intakeTimer.seconds() > 1) {
                    intakeMotor.intake();
                    intakeServos.intakePosition();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_WAITFORBLOCK;
                }
                if (gamepad1.dpad_left) {
                    intakeMotor.outtake();
                }

                break;

            case INTAKE_WAITFORBLOCK:
                colorAndDistance.update();

                String detectedColor = colorAndDistance.getDetectedColor();

                if (detectedColor.equals("Red") || detectedColor.equals("Yellow")) {
                    intakeMotor.stop();
                    intakeServos.transferPosition();
                    intakeTimer.reset();
                    intakeState = IntakeState.INTAKE_RETRACT;
                }
                if (gamepad1.dpad_left) {
                    intakeMotor.outtake();
                }


                break;

            case INTAKE_RETRACT:
                linkageController.setPosition(LinkageController.Position.RETRACTED);

                if (gamepad1.right_bumper) {
                    intakeTimer.reset();
                    intakeState = IntakeState.OUTAKE_HUMAIN;
                } else if (gamepad1.left_bumper) {
                    intakeTimer.reset();
                    intakeState = IntakeState.OUTAKE_BLOCK;

                }

                if (gamepad1.dpad_up) {
                    intakeState = IntakeState.INTAKE_EXTEND;
                }

                break;

            case OUTAKE_HUMAIN:
                colorAndDistance.update();
                if (intakeTimer.seconds() < 0.75) {
                    intakeMotor.outtake();
                }

                if (intakeTimer.seconds() > 0.75) {
                    intakeState = IntakeState.INTAKE_START;
                }


                break;

            case OUTAKE_BLOCK:
                if (intakeTimer.seconds() < 1.5) {
                    intakeMotor.intake();

                }

                if (intakeTimer.seconds() > 1.5) {
                    intakeState = IntakeState.INTAKE_START;

                }


                break;

            default:
                // should never be reached, as intakeState should never be null
                intakeState = IntakeState.INTAKE_START;

        }

        if (gamepad1.dpad_down && intakeState != IntakeState.INTAKE_START) {
            intakeState = IntakeState.INTAKE_START;
        }
        // Measure loop time
        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();

        // TeleOp movement
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        // Update Color Sensor
        colorAndDistance.update();

        String detectedColor = colorAndDistance.getDetectedColor();

        // Rumble for Blue Detection
        if ((detectedColor.equals("Red") && !hasRumbled)) {
            gamepad1.rumble(1000);
            hasRumbled = true;
        }
        if (!detectedColor.equals("Red") && !detectedColor.equals("Yellow")) {
            hasRumbled = false;
        }

        if (viperSlides.getSlidePositionRight() > 1300) {
            if (gamepad1.right_trigger > 0.1) {
                bucketServos.depositPosition(); // Move bucket to deposit position if right trigger is pressed and slides are down
            } else {
                bucketServos.transferPosition();   // Otherwise, set bucket transfer position
            }
        } else {
            bucketServos.transferPosition();       // If the slide position is not less than -1950, set bucket to transfer position
        }

        currentLeftTriggerState = gamepad1.left_trigger > 0.5;  // Detect if the left trigger is pressed
        if (currentLeftTriggerState && !previousLeftTriggerState) {  // Rising edge
            // Toggle claw position on rising edge
            if (isClawOpen) {
                clawServo.closedPosition();  // Close the claw
            } else {
                clawServo.openPosition();  // Open the claw
            }
            // Flip the claw state
            isClawOpen = !isClawOpen;
        }
        previousLeftTriggerState = currentLeftTriggerState;


        linkageController.checkForAmperageSpike();
        linkageController.update();

        // Viper Slide Control (Predefined Targets)
        viperSlides.update();

        if (viperSlides.isLimitSwitchPressed() && !wasLimitSwitchPressed) {
            viperSlides.resetPosition();
        }

        wasLimitSwitchPressed = viperSlides.isLimitSwitchPressed();

        if (gamepad1.y) {
            viperSlides.setTarget(ViperSlides.Target.HIGH);
        }
        if (gamepad1.a && bucketServos.isTransferPosition() && isClawOpen) {
            viperSlides.setTarget(ViperSlides.Target.GROUND);
        }
        if (gamepad1.b && bucketServos.isTransferPosition()) {
            viperSlides.setTarget(ViperSlides.Target.LOW);
        }
        if (gamepad1.x && bucketServos.isTransferPosition()) {
            viperSlides.setTarget(ViperSlides.Target.MEDIUM);

        }

        if (gamepad1.touchpad) {
            viperSlides.setTarget(ViperSlides.Target.LEVEL1);
        }

        // Home Slides
        if (gamepad1.options) {
            viperSlides.setPIDEnabled(false);

            while (!viperSlides.isLimitSwitchPressed()) {
                viperSlides.setSlidePower(-1.0);
            }

            viperSlides.setSlidePower(0);
            viperSlides.resetPosition();
            viperSlides.setPIDEnabled(true);
        }

        if (gamepad1.back) {
            Pose currentPose = follower.getPose();
            Pose newPose = new Pose(currentPose.getX(), currentPose.getY(), 0);
            follower.setPose(newPose);
        }

        // Telemetry for debugging and visualization
        telemetry.addData("Loop Time (ms)", loopTime);  // Show the loop time in ms
        telemetry.addData("Slide Position Left", viperSlides.getSlidePositionLeft());
        telemetry.addData("Slide Position Right", viperSlides.getSlidePositionRight());
        telemetry.addData("Slide Target", viperSlides.getTarget());
        telemetry.addData("Detected Color", colorAndDistance.getDetectedColor());
        //
        telemetry.addData("Current Position", linkageController.getCurrentPosition());
        telemetry.addData("Target Position", linkageController.getTargetPosition());
        telemetry.addData("At Target", linkageController.isAtTarget());
        telemetry.addData("Is Extended", linkageController.isExtended());
        telemetry.addData("Is Retracted", linkageController.isRetracted());
        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("intake State", intakeState);
        telemetry.update();

    }
}